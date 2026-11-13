#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>

cv::CascadeClassifier face_cascade_;

// The depth image must already be registered to the rectified RGB image.
// This node decompresses images; it does not perform registration or rectification.
class RgbdCloud : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using Info = sensor_msgs::msg::CameraInfo;
  using Policy = message_filters::sync_policies::ApproximateTime<Image, Image, Info>;

public:
  RgbdCloud() : Node("rgbd_cloud_demo")
  {
    const auto rgb_topic = declare_parameter<std::string>(
      "rgb_topic", "/head_rgbd_sensor/rgb/image_rect_color");
    const auto depth_topic = declare_parameter<std::string>(
      "depth_topic", "/head_rgbd_sensor/depth_registered/image");
    const auto info_topic = declare_parameter<std::string>(
      "camera_info_topic", "/head_rgbd_sensor/rgb/camera_info");
    const auto output_topic = declare_parameter<std::string>(
      "points_topic", "/head_rgbd_sensor/depth_registered/demo_points");
    const auto cascade_path = ament_index_cpp::get_package_share_directory("rgbd_cloud_demo") +
      "/config/haarcascade_frontalface_default.xml";

    const int queue = declare_parameter<int>("sync_queue_size", 30);
    const double tolerance_ms = declare_parameter<double>("max_sync_interval_ms", 50.0);

    //Condition check
    if (queue < 2 || queue > 1000 || !std::isfinite(tolerance_ms) || tolerance_ms < 0.0) {
      throw std::invalid_argument("Use sync_queue_size=2..1000 and a finite, nonnegative tolerance");
    }
    if (!face_cascade_.load(cascade_path)) {
      throw std::runtime_error("faild to load face cascade");
    }

    auto qos = rmw_qos_profile_sensor_data;
    qos.depth = static_cast<size_t>(queue);
    // SubscriberFilter handles decompression and feeds decoded Image messages
    // directly to message_filters. No intermediate raw topic is published.
    rgb_sub_.subscribe(this, rgb_topic, "compressed", qos);
    depth_sub_.subscribe(this, depth_topic, "compressedDepth", qos);
    info_sub_.subscribe(this, info_topic, qos);

    rgb_sub_.registerCallback([this](const Image::ConstSharedPtr &) {++rgb_count_;});
    depth_sub_.registerCallback([this](const Image::ConstSharedPtr &) {++depth_count_;});
    info_sub_.registerCallback([this](const Info::ConstSharedPtr &) {++info_count_;});

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, rclcpp::SensorDataQoS());

    sync_ = std::make_shared<message_filters::Synchronizer<Policy>>(Policy(queue), rgb_sub_, depth_sub_, info_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(tolerance_ms / 1000.0));
    sync_->registerCallback(std::bind(
      &RgbdCloud::process, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3
      ));

    // Wall time keeps diagnostics useful even while bag /clock is paused.
    diagnostics_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO(get_logger(),
        "Received decoded RGB=%llu depth=%llu CameraInfo=%llu; published clouds=%llu",
        static_cast<unsigned long long>(rgb_count_),
        static_cast<unsigned long long>(depth_count_),
        static_cast<unsigned long long>(info_count_),
        static_cast<unsigned long long>(cloud_count_));
    });
    RCLCPP_INFO(get_logger(), "RGB input: %s", rgb_sub_.getTopic().c_str());
    RCLCPP_INFO(get_logger(), "Depth input: %s", depth_sub_.getTopic().c_str());
    RCLCPP_INFO(get_logger(), "PointCloud2 output: %s", output_topic.c_str());
  }

private:
  void process(
    const Image::ConstSharedPtr & rgb,
    const Image::ConstSharedPtr & depth,
    const Info::ConstSharedPtr & info)
  {
    try {
      const auto color = cv_bridge::toCvShare(rgb, "bgr8");
      const auto distance = cv_bridge::toCvShare(depth);  // Preserve numerical depth.
      show_rgb(color->image);
      if (!validate_inputs(*rgb, *depth, *info, color->image, distance->image)) {
        return;
      }

      const auto cloud = create_point_cloud(color->image, distance->image, *depth, *info);
      publisher_->publish(cloud);
      ++cloud_count_;
      log_published_cloud(cloud, rgb, depth, info);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "Point cloud conversion failed: %s", error.what());
    }
  }

  void show_rgb(const cv::Mat & color)
  {
    if (!color.empty()) {
      cv::Mat gray, display;
      cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
      cv::equalizeHist(gray, gray);
      display = color.clone();

      std::vector<cv::Rect> faces;
      face_cascade_.detectMultiScale(
          gray, faces
          );

      for(const auto & face : faces){
        cv::rectangle(display, face, cv::Scalar(0, 255, 0), 2);

      }

      for(const auto&face : faces){
      //displays tracking red-dot
        cv::Point face_center(face.x + face.width/2, face.y + face.height /2);
        cv::circle(display, face_center, 4, cv::Scalar(0, 0, 255), cv::FILLED);
      }

      cv::imshow("RGB", display);
      cv::waitKey(1);
    }
  }

  bool validate_inputs(
    const Image & rgb, const Image & depth, const Info & info,
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

    // P describes the rectified RGB camera. XYZ is relative to that camera.
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
    const Image & depth, const Info & info)
  {
    const bool millimeters = depth.encoding == sensor_msgs::image_encodings::TYPE_16UC1;
    const double fx = info.p[0], fy = info.p[5];
    const double cx = info.p[2], cy = info.p[6];
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = depth.header;
    cloud.width = depth.width;
    cloud.height = depth.height;
    cloud.is_dense = false;  // invalid depths become nan.
    cloud.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    // setPointCloud2FieldsByString already allocates width * height points.
    // do not resize here: humble's resize flattens organized clouds to one row.
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

  void log_published_cloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const Image::ConstSharedPtr & rgb,
    const Image::ConstSharedPtr & depth,
    const Info::ConstSharedPtr & info)
  {
    const auto rgb_ns = rclcpp::Time(rgb->header.stamp).nanoseconds();
    const auto depth_ns = rclcpp::Time(depth->header.stamp).nanoseconds();
    const auto info_ns = rclcpp::Time(info->header.stamp).nanoseconds();
    const double span_ms = (std::max({rgb_ns, depth_ns, info_ns}) -
      std::min({rgb_ns, depth_ns, info_ns})) / 1e6;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Published %ux%u XYZRGB cloud; frame=%s; timestamp span=%.3f ms",
      cloud.width, cloud.height, cloud.header.frame_id.c_str(), span_ms);
  }

  image_transport::SubscriberFilter rgb_sub_, depth_sub_;
  message_filters::Subscriber<Info> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr diagnostics_;
  uint64_t rgb_count_{0}, depth_count_{0}, info_count_{0}, cloud_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RgbdCloud>());
  rclcpp::shutdown();
  return 0;
}
