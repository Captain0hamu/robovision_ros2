#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class ImageProcessingNode : public rclcpp::Node
{
public:
    ImageProcessingNode() : Node("image_processing"), is_image_(false)
    {
        counter_ = 0;

        //Subscribers
        image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", 10, std::bind(&ImageProcessingNode::callback_image, this, std::placeholders::_1));
        
        //Processing
        image_processing_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ImageProcessingNode::image_processing, this));

        RCLCPP_INFO(this->get_logger(), "Starting image_processing application in cpp...");
    }

private:

    bool is_image_;
    int counter_;
    cv::Mat image_, image_gray;
    cv::Mat mask;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::TimerBase::SharedPtr image_processing_timer_;
    
    void callback_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        is_image_ = true;
    }

    void image_processing()
    {
        if (is_image_){


                /*
            if (counter_ <= 9){
                int row_id, col_id;
                row_id = image_.rows/2 + (counter_ + counter_);
                col_id = image_.cols/2 + (counter_ + counter_);

                std::cout << "size: rows: " << image_.rows << 
                               ", cols: " << image_.cols << 
                               ", depth: " << image_.channels() << std::endl;

                std::cout << "size: rows: " << image_gray.rows << 
                               ", cols: " << image_gray.cols << 
                               ", depth: " << image_gray.channels() << std::endl;
                //alignment
                std::cout << "\n";

                std::cout << "\033[" << 31 + counter_%6 <<"m" << "attempt" << counter_ +1 << "\033[0m\n";

                cv::vec3b pixel = image_.at<cv::vec3b>(row_id, col_id);

                std::cout << "pixel value in img at row=" << row_id <<
                              ", col=" << col_id <<
                              " is: " << pixel << "\n\n";
                
                std::cout << "pixel value in img_gray at row=" << row_id <<
                              ", col=" << col_id <<
                              " is: " << (int)image_gray.at<uchar>(row_id, col_id) << "\n";

                int gray_calc = 
                  0.11 * pixel[0] +
                  0.59 * pixel[1] +
                  0.30 * pixel[2];

                std::cout << "Gray(calculated)=" << gray_calc << "\n";

            }
                */
        cv::cvtColor(image_, image_gray, CV_BGR2GRAY);

        cv::Mat image_gray_2 = cv::Mat::zeros(image_.rows, image_.cols, CV_8UC1);
        for (int i = 0; i < image_.rows; i++){
          for (int j = 0; j < image_.cols; j++){
            cv::Vec3b pixel = image_.at<cv::Vec3b>(image_.rows-1-i, image_.cols-1-j);

            int gray_val = pixel[0] * 0.11 + pixel[1] * 0.59 + pixel[2] * 0.30;
            image_gray_2.at<uchar>(i,j) = (unsigned char)gray_val;
          }
        }
        cv::imshow("gray_2", image_gray_2);

        cv::inRange(image_, cv::Scalar(0, 0, 220), cv::Scalar(240, 255, 240), mask);

        cv::Mat image_filtered;
        image_.copyTo(image_filtered, mask);

        sensor_msgs::msg::Image::SharedPtr image_msg_;

        cv::putText(image_, std::to_string(counter_), cv::Point(25, 25),
                cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(0, 255, 0), 2);


        cv::imshow("view", image_);
        //cv::imshow("gray", image_gray);
        cv::imshow("mask", mask);
        cv::imshow("image_filtered", image_filtered);
        cv::waitKey(1);

            counter_++;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
