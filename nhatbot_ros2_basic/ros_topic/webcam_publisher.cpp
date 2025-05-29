#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class MinimalImagePublisher : public rclcpp::Node
{
    public:
        MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0)
        {
            publisher_ = 
                this->create_publisher<sensor_msgs::msg::Image>("random_image", 10);
            timer_ = this->create_wall_timer(
                500ms, std::bind(&MinimalImagePublisher::timer_callback, this));      
        }
    private:
        void timer_callback()
        {
            cv::Mat my_image(cv::Size(640,480), CV_8UC3);
            cv::randu(my_image, cv::Scalar(0,0,0), cv::Scalar(255,255,255)); // random pixels all of the image
            
            cv::Point text_position(15, 40);
            int font_size = 1;
            cv::Scalar font_color(255,255,255);
            int font_weight = 2;
            cv::putText(my_image, "ROS2 + OpenCV", text_position, cv::FONT_HERSHEY_COMPLEX, font_size, font_color, font_weight);
            
            
            msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", my_image).toImageMsg();
            publisher_ -> publish(*msg_.get());
            RCLCPP_INFO(this->get_logger(),"Image %ld published", count_);
            count_++;
        }
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::Image::SharedPtr msg_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
