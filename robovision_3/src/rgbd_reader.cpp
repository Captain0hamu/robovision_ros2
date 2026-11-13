#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <cstddef>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/head_rgbd_sensor/rgb/image_rect_color", 10, 
            std::bind(&PointCloudCentroidNode::callback_rgb_rect, this, std::placeholders::_1));
        
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/head_rgbd_sensor/depth_registered/image", 10, 
            std::bind(&PointCloudCentroidNode::callback_depth_rect, this, std::placeholders::_1));
        
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/head_rgbd_sensor/depth_registered/camera_info", 10, 
            std::bind(&PointCloudCentroidNode::callback_point_cloud, this, std::placeholders::_1));
        
        // Processing
        processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&PointCloudCentroidNode::point_cloud_processing, this));

        RCLCPP_INFO(this->get_logger(), "starting point_cloud_centroid application in cpp...");
    }

private:

    bool is_ptcld_;
    bool display_;
    cv::Mat rgb_, depth_, depth_mat_, point_cloud_, mask;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr centroid_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;

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
            RCLCPP_ERROR(this->get_logger(), "error processing rgb image: %s", e.what());
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

    /*
    void detect(cv::Mat &frame)
    {
      if(mask.empty()) return;

      int top = mask.rows;
      int buttom = 0;
      int right = 0;
      int left = mask.cols;
      //左から順に見つけていく
      for (int y = 0; y < mask.rows; y++)
      {
        const uchar* row_ptr = mask.ptr<uchar>(y);
        for (int x = 0; x < mask.cols; x++)
        {
          if(row_ptr[x] == 255)
          {
            if(left > x) left = x;
            if(right < x) right = x;
            if(top > y) top = y;
            if(buttom < y) buttom = y;
          }
        }
      }

      int middle_point[2];
      //width = left - right + 1;
      //height = top - buttom + 1;
      middle_point[0] = (left - right + 1) / 2; // x
      middle_point[1] = (top - buttom + 1) / 2; // y

      cv::Point center;
      center.x = middle_point[0];
      center.y = middle_point[1];

      cv::circle(frame, center, 5, cv::Scalar(0, 255, 0), -1);

    }
    */

    void point_cloud_processing() 
    {
        if (!is_ptcld_) {
            RCLCPP_WARN(this->get_logger(), "Empty point cloud matrix.");
            return;
        }

        const int rows = point_cloud_.rows;
        const int cols = point_cloud_.cols;

        // cv::Point(x, y): x is the column and y is the row.
        const std::vector<cv::Point> sample_points = {
            cv::Point(cols / 2, rows / 2),
            cv::Point(cols / 4, rows / 2),
            cv::Point(cols / 2, rows / 4),
            cv::Point(3 * cols / 4, rows / 2),
            cv::Point(cols / 2, 3 * rows / 4)
        };

        std::cout << "depth_ channels: " << depth_.channels() << std::endl;
        std::cout << "depth_ type (depth): " << depth_.depth() << " (CV_8U=" << CV_8U << ", CV_16U=" << CV_16U << ", CV_32F=" << CV_32F << ")" << std::endl;
        cv::Mat debug_rgb = rgb_.clone(); 

        double minVal, maxVal;
        cv::minMaxLoc(depth_, &minVal, &maxVal);
        std::cout << "depth_ min: " << minVal << ", max: " << maxVal << std::endl;

        cv::inRange(depth_, cv::Scalar(0.1f), cv::Scalar(0.5f), mask);
        cv::Mat result;
        cv::bitwise_and(depth_, depth_, result, mask);

        for (std::size_t i = 0; i < sample_points.size(); ++i) {
            const int row_id = sample_points[i].y;
            const int col_id = sample_points[i].x;

            if (row_id < 0 || row_id >= rows || col_id < 0 || col_id >= cols) {
                RCLCPP_WARN(this->get_logger(), "PointCloud index out of bounds");
                continue;
            }

            const cv::Vec4f point = point_cloud_.at<cv::Vec4f>(row_id, col_id);
            const float x = point[0];
            const float y = point[1];
            const float z = point[2];

            geometry_msgs::msg::Pose centroid;
            centroid.position.x = x;
            centroid.position.y = y;
            centroid.position.z = z;

            tf2::Quaternion q;
            const float roll = 0.0;
            //const float pitch = ;
            //const float yaw = ;
            //q.setRPY(roll, pitch, yaw);

            centroid.orientation.x = q.x();
            centroid.orientation.y = q.y();
            centroid.orientation.z = q.z();
            centroid.orientation.w = q.w();

            centroid_publisher_->publish(centroid);
            
            RCLCPP_INFO(
                this->get_logger(),
                "P%zu pixel=(%d, %d), xyz=(%.3f, %.3f, %.3f)",
                i, col_id, row_id, x, y, z);
            centroid_publisher_->publish(centroid);

            if (!debug_rgb.empty()) {
                cv::circle(debug_rgb, cv::Point(col_id, row_id), 6,
                           cv::Scalar(0, 0, 255), -1);
                cv::putText(debug_rgb, "P" + std::to_string(i),
                            cv::Point(col_id + 8, row_id - 8),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            cv::Scalar(0, 0, 255), 1);

                const std::string xyz = "xyz=(" + cv::format("%.2f", x) + "," +
                                        cv::format("%.2f", y) + "," +
                                        cv::format("%.2f", z) + ")";
                cv::putText(debug_rgb, xyz,

                            cv::Point(col_id + 8, row_id + 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4,
                            cv::Scalar(0, 0, 255), 1);
            }
        }

        if (display_ && !debug_rgb.empty()) {
            //detect(debug_rgb);
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
