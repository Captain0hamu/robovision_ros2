#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class PointCloudCentroidSyncNode : public rclcpp::Node
{
public:
    using cf = const float;
    using Image = sensor_msgs::msg::Image;
    using CameraInfo = sensor_msgs::msg::CameraInfo;
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        Image, Image, CameraInfo>;

    PointCloudCentroidSyncNode()
    : Node("point_cloud_centroid")
    {
        const auto cascade_path = this->declare_parameter<std::string>(
            "face_cascade_path",
            ament_index_cpp::get_package_share_directory("robovision_rgbd") +
            "/config/haarcascade_frontalface_default.xml");
        if (!face_cascade_.load(cascade_path)) {
            throw std::runtime_error("Failed to load face cascade: " + cascade_path);
        }

        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/object_centroid", 10);
        point_cloud_publisher_ = this->create_publisher<PointCloud2>(
            "/camera/depth_registered/synced_points", rclcpp::SensorDataQoS());

        const int sync_queue_size = this->declare_parameter<int>("sync_queue_size", 100);
        const double max_interval_ms =
            this->declare_parameter<double>("max_sync_interval_ms", 50.0);
        if (sync_queue_size < 2 || sync_queue_size > 1000 ||
            !std::isfinite(max_interval_ms) || max_interval_ms < 0.0)
        {
            throw std::invalid_argument(
                "Use sync_queue_size=2..1000 and a finite, nonnegative max_sync_interval_ms");
        }

        const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(sync_queue_size);
        rgb_sub_.subscribe(
            this, "/camera/rgb/image_rect_color", sensor_qos.get_rmw_qos_profile());
        depth_sub_.subscribe(
            this, "/camera/depth_registered/image", sensor_qos.get_rmw_qos_profile());
        camera_info_sub_.subscribe(
            this, "/camera/rgb/camera_info", sensor_qos.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(sync_queue_size), rgb_sub_, depth_sub_, camera_info_sub_);
        sync_->setMaxIntervalDuration(
            rclcpp::Duration::from_seconds(max_interval_ms / 1000.0));
        sync_->registerCallback(
            std::bind(
                &PointCloudCentroidSyncNode::callback_rgb_depth,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3));

        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&PointCloudCentroidSyncNode::point_cloud_processing, this));

        RCLCPP_INFO(this->get_logger(),
            "Synchronizing RGB/depth/CameraInfo; publishing /camera/depth_registered/synced_points");
    }

private:
    cv::CascadeClassifier face_cascade_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr centroid_publisher_;
    rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_publisher_;
    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;
    message_filters::Subscriber<CameraInfo> camera_info_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::TimerBase::SharedPtr processing_timer_;
    PointCloud2 point_cloud_;

    // 顔画像と点群は、必ずこのcallbackの同じ入力から作る。
    void callback_rgb_depth(
        const Image::ConstSharedPtr & rgb_msg,
        const Image::ConstSharedPtr & depth_msg,
        const CameraInfo::ConstSharedPtr & info_msg)
    {
        try {
            const auto rgb_ptr = cv_bridge::toCvCopy(rgb_msg, "bgr8");
            const auto depth_ptr = cv_bridge::toCvShare(depth_msg);
            cv::Mat & image = rgb_ptr->image;
            const cv::Mat & depth = depth_ptr->image;
            if (!validate_inputs(*rgb_msg, *depth_msg, *info_msg, image, depth)) {
                return;
            }

            // 枠の色が点群へ混ざらないよう、描画前のRGBを使う。
            auto cloud = create_point_cloud(image, depth, *depth_msg, *info_msg);
            show_rgb(image, cloud);
            point_cloud_publisher_->publish(cloud);
            point_cloud_ = std::move(cloud);

            const auto rgb_stamp = rclcpp::Time(rgb_msg->header.stamp).nanoseconds();
            const auto depth_stamp = rclcpp::Time(depth_msg->header.stamp).nanoseconds();
            const auto info_stamp = rclcpp::Time(info_msg->header.stamp).nanoseconds();
            const double span_ms =
                (std::max({rgb_stamp, depth_stamp, info_stamp}) -
                 std::min({rgb_stamp, depth_stamp, info_stamp})) / 1e6;
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Generated synchronized XYZRGB: %ux%u, span=%.3f ms",
                point_cloud_.width, point_cloud_.height, span_ms);
            RCLCPP_DEBUG(
                this->get_logger(), "Sync stamps: rgb=%lld depth=%lld info=%lld",
                static_cast<long long>(rgb_stamp), static_cast<long long>(depth_stamp),
                static_cast<long long>(info_stamp));
        } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Synchronized RGB/depth processing failed: %s", e.what());
        }
    }

    void show_rgb(cv::Mat & image, const PointCloud2 & cloud)
    {
        using PointIterator = sensor_msgs::PointCloud2ConstIterator<float>;
        cv::Mat gray, display;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray, gray);
        display = image.clone();

        std::vector<cv::Rect> faces;
        face_cascade_.detectMultiScale(gray, faces);

        PointIterator x(cloud, "x");
        PointIterator y(cloud, "y");
        PointIterator z(cloud, "z");

        for (const auto & face : faces) {
            cv::rectangle(display, face, cv::Scalar(0, 255, 0), 2);

            const int u_mid = face.x + face.width / 2;
            const int v_mid = face.y + face.height / 2;
            const cv::Point face_center(u_mid, v_mid);
            cv::circle(display, face_center, 4, cv::Scalar(0, 0, 255), cv::FILLED);

            const int center_idx = v_mid * cloud.width + u_mid;

            cf center_x = *(x + center_idx);
            cf center_y = *(y + center_idx);
            /* cf center_z = *(z + center_idx);
            cf distance_from_center = sqrt(
                center_x*center_x + \
                center_y*center_y + \
                center_z*center_z);  */

            int count = 0;
            float sum = 0.0;
            float distance_from_av;

            for (auto v = face.y; v < face.y + face.height; ++v){
              for (auto u = face.x; u < face.x + face.width; ++u){
                const int index = v * cloud.width + u;
                float av_depth = *(z + index);

                if (std::isfinite(av_depth) && av_depth > 0.0f){
                  count++;
                  sum += av_depth;
                }

                av_depth = sum / count;
                distance_from_av = sqrt(
                  center_x*center_x + \
                  center_y*center_y + \
                  av_depth*av_depth);}
            }
            /*
            if (!std::isnan(center_z)){
              cv::putText(display, "Distance from face(center depth)" + std::to_string(distance_from_center)+"m",
                cv::Point((int)(face.width), (int)(face.height - 8)),
                cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(255, 0, 0), 1);
            }else{
            */
            cv::putText(display, "Distance from face(average depth)" + std::to_string(distance_from_av)+"m",
              cv::Point((int)(face.width), (int)(face.height - 8)),
              cv::FONT_HERSHEY_SIMPLEX, 0.8,
              cv::Scalar(255, 0, 0), 1);
            //}
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "RGB face detection: %zu faces (%dx%d)",
            faces.size(), display.cols, display.rows);
        cv::imshow("face detection window", display);
        cv::waitKey(1);
    }

    bool validate_inputs(
        const Image & rgb, const Image & depth, const CameraInfo & info,
        const cv::Mat & color, const cv::Mat & distance)
    {
        if (color.empty() || distance.empty() ||
            color.size() != distance.size() ||
            info.width != rgb.width || info.height != rgb.height)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Empty image or mismatched RGB/depth/CameraInfo dimensions; skipping");
            return false;
        }
        const bool millimeters = depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1;
        if (!millimeters && depth.encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Unsupported depth encoding: %s (expected 16UC1 or 32FC1)", depth.encoding.c_str());
            return false;
        }

        const double fx = info.p[0], fy = info.p[5];
        const double cx = info.p[2], cy = info.p[6];
        if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0 || fy <= 0 ||
            !std::isfinite(cx) || !std::isfinite(cy))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Invalid CameraInfo projection matrix");
            return false;
        }
        if (depth.header.frame_id.empty() ||
            depth.header.frame_id != rgb.header.frame_id ||
            rgb.header.frame_id != info.header.frame_id)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Frame IDs differ or are empty; registered RGB/depth/CameraInfo are required");
            return false;
        }
        return true;
    }

    sensor_msgs::msg::PointCloud2 create_point_cloud(
        const cv::Mat & color, const cv::Mat & distance,
        const Image & depth, const CameraInfo & info)
    {
        const bool millimeters = depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1;
        const double fx = info.p[0], fy = info.p[5];
        const double cx = info.p[2], cy = info.p[6];
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = depth.header;
        cloud.width = depth.width;
        cloud.height = depth.height;
        cloud.is_dense = false;
        cloud.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> r(cloud, "r"), g(cloud, "g"), b(cloud, "b");
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (int v = 0; v < distance.rows; ++v) {
            const auto * colors = color.ptr<cv::Vec3b>(v);
            for (int u = 0; u < distance.cols; ++u, ++x, ++y, ++z, ++r, ++g, ++b) {
                const float meters = millimeters ?
                    distance.at<uint16_t>(v, u) * 0.001f :
                    distance.at<float>(v, u);
                if (std::isfinite(meters) && meters > 0) {
                    *x = static_cast<float>((u - cx) * meters / fx);
                    *y = static_cast<float>((v - cy) * meters / fy);
                    *z = meters;
                } else {
                    *x = *y = *z = nan;
                }
                *b = colors[u][0];
                *g = colors[u][1];
                *r = colors[u][2];
            }
        }
        return cloud;
    }


    void point_cloud_processing()
    {
        if (point_cloud_.data.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Waiting for synchronized RGB/depth/CameraInfo");
            return;
        }

        const int rows = point_cloud_.height;
        const int cols = point_cloud_.width;
        const std::vector<cv::Point> sample_points = {
            cv::Point(cols / 2, rows / 2),
            cv::Point(cols / 4, rows / 2),
            cv::Point(cols / 2, rows / 4),
            cv::Point(3 * cols / 4, rows / 2),
            cv::Point(cols / 2, 3 * rows / 4)};

        for (const auto & pixel : sample_points) {
            const int index = pixel.y * cols + pixel.x;
            sensor_msgs::PointCloud2ConstIterator<float> x(point_cloud_, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y(point_cloud_, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z(point_cloud_, "z");

            geometry_msgs::msg::Pose centroid;
            centroid.position.x = *(x + index);
            centroid.position.y = *(y + index);
            centroid.position.z = *(z + index);
            centroid.orientation.w = 1.0;
            centroid_publisher_->publish(centroid);
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudCentroidSyncNode>();
    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
