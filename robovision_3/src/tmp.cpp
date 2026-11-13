#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <cstddef>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <string>
#include <vector>
#include <exception>


class PointCloudCentroidNode : public rclcpp::Node
{
public:
    PointCloudCentroidNode() : Node("point_cloud_centroid"), is_ptcld_(false), display_(true)
    {
        // Publishers
        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
            "/object_centroid", 10);

        // Subscribers
        rgb_sub_.subscribe( this, "/camera/rgb/image_rect_color");

        depth_sub_.subscribe( this, "/camera/depth_registered/image");

        sync_ = std::make_shared< message_filters::Synchronizer<SyncPolicy> >( SyncPolicy(10), rgb_sub_, depth_sub_);

        sync_->registerCallback(
            std::bind(
                &PointCloudCentroidNode::sync_callback,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        
        //point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", 10, 
            std::bind(&PointCloudCentroidNode::callback_point_cloud, this, std::placeholders::_1);
        
        // Processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&PointCloudCentroidNode::point_cloud_processing, this));

        RCLCPP_INFO(this->get_logger(), "Starting point_cloud_centroid application in cpp...");
    }


private:
    using Image = sensor_msgs::msg::Image;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

    message_filters::Subscriber<Image> rgb_sub_;
    message_filters::Subscriber<Image> depth_sub_;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    bool is_ptcld_;
    bool display_;
    cv::Mat rgb_, depth_, depth_mat_, point_cloud_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr centroid_publisher_;

    //Subscribers

    rclcpp::TimerBase::SharedPtr processing_timer_;

    void callback_rgb_rect(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        try 
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            rgb_ = cv_ptr->image.clone();
            cv::cvtColor(rgb_, rgb_, cv::COLOR_RGB2BGR);
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing RGB image: %s", e.what());
        }
    }

    void callback_depth_rect(const sensor_msgs::msg::Image::SharedPtr msg) 
    {
        try 
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
            depth_ = cv_ptr->image.clone();

            depth_mat_ = cv::Mat(depth_.size(), CV_32F);
            depth_.convertTo(depth_mat_, CV_32F);
            cv::normalize(depth_mat_, depth_, 0, 1, cv::NORM_MINMAX);
        } 
        catch (const std::exception &e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error processing Depth image: %s", e.what());
        }
    }

    void callback_point_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
    {
        is_ptcld_ = false;
        try
        {
            // Ensure the point cloud is organized
            if (msg->height > 1)
            {
                // Convert PointCloud2 to cv::Mat
                point_cloud_ = cv::Mat(msg->height, msg->width, CV_32FC4);
                const uint8_t *data_ptr = msg->data.data();

                for (size_t i = 0; i < msg->height; ++i) 
                {
                    for (size_t j = 0; j < msg->width; ++j) 
                    {
                        const float *point = reinterpret_cast<const float*>(data_ptr + (i * msg->row_step + j * msg->point_step));
                        point_cloud_.at<cv::Vec4f>(i, j) = cv::Vec4f(point[0], point[1], point[2], point[3]);
                    }
                }

                is_ptcld_ = true;
                RCLCPP_INFO(this->get_logger(), "Point cloud converted to cv::Mat with size [%d, %d]", msg->height, msg->width);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Point cloud is not organized. Cannot convert to cv::Mat.");
            }
        }
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(this->get_logger(), "Error converting point cloud: %s", e.what());
        }
    }

    void sync_callback( const Image::ConstSharedPtr& rgb_msg, const Image::ConstSharedPtr& depth_msg)
    {
        try
        {
            // RGB画像
            auto rgb_ptr = cv_bridge::toCvCopy(
                rgb_msg, "bgr8"
            );

            cv::Mat debug_rgb = rgb_ptr->image;

            // Depth画像（元のエンコーディングを維持）
            auto depth_ptr = cv_bridge::toCvShare(depth_msg);

            cv::Mat depth;

            if (depth_msg->encoding == "32FC1")
            {
                // 通常はメートル単位
                depth = depth_ptr->image;
            }
            else if (depth_msg->encoding == "16UC1")
            {
                // 通常はミリメートル単位
                depth_ptr->image.convertTo(
                    depth,
                    CV_32F,
                    0.001
                );
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Unsupported depth encoding: %s",
                    depth_msg->encoding.c_str()
                );
                return;
            }

            if (debug_rgb.size() != depth.size())
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "RGB/Depth resolution mismatch"
                );
                return;
            }

            detect(depth, debug_rgb);

            cv::imshow("face detection window", debug_rgb);
            cv::waitKey(1);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Image processing failed: %s",
                e.what()
            );
        }
    }

    void detect(const cv::Mat& depth, cv::Mat& debug_rgb) {
        if (depth.empty() || debug_rgb.empty())
            return;

        cv::Mat mask;

        // 深度が0.5m〜1.5mの画素を抽出
        cv::inRange( depth, cv::Scalar(0.5f), cv::Scalar(1.5f), mask);

        // 各連結領域にラベルを付与
        cv::Mat labels, stats, centroids;

        int count = cv::connectedComponentsWithStats( mask, labels, stats, centroids, 8, CV_32S);

        int largest_id = -1;
        int largest_area = 100; // 小さい領域を除外

        // ID 0は背景なので1から開始
        for (int i = 1; i < count; i++)
        {
            int area = stats.at<int>(
                i, cv::CC_STAT_AREA
            );

            if (area > largest_area)
            {
                largest_area = area;
                largest_id = i;
            }
        }

        if (largest_id == -1)
            return;

        // 最大領域の重心
        int center_x = static_cast<int>(
            centroids.at<double>(largest_id, 0)
        );

        int center_y = static_cast<int>(
            centroids.at<double>(largest_id, 1)
        );

        cv::Point center(center_x, center_y);

        cv::circle( debug_rgb, center, 5, cv::Scalar(0, 255, 0), -1);
    }

    void point_cloud_processing() 
    {
        if (!is_ptcld_) {
            RCLCPP_WARN(this->get_logger(), "Empty point cloud matrix.");
            return;
        }

        const int rows = point_cloud_.rows;
        const int cols = point_cloud_.cols;

        cv::Mat debug_rgb = rgb_.clone(); 

        double minVal, maxVal;
        cv::minMaxLoc(depth_, &minVal, &maxVal);
        std::cout << "depth_ min: " << minVal << ", max: " << maxVal << std::endl;


        if (display_ && !debug_rgb.empty()) {
            detect(depth_, debug_rgb);
            cv::imshow("face detection window", debug_rgb);
            if (!depth_.empty()) {
                cv::imshow("Depth Image", depth_);
            }
            cv::waitKey(1);
        }
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudCentroidNode>();
    rclcpp::spin(node);

    cv::destroyAllWindows();
    rclcpp::shutdown();
    
    return 0;
}
