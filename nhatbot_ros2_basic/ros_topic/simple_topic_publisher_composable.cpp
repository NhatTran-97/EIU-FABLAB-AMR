

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "chrono"

using namespace std::chrono_literals;

/*This example creates a subclass of Node and uses std::bind() to register a  *member function as a callback from the timer*/

class SimplePublisher:public rclcpp::Node
{
public: 
        SimplePublisher(): Node("publisher_node"),count_(0)
        {
            //message = std::make_shared<std_msgs::msg::Int32>(); message->data = 0;

            publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter_2", 10);

            //timer_ = this -> create_wall_timer(500ms, std::bind(&SimplePublisher::timer_callback, this));
            timer_ = this -> create_wall_timer(500ms, [this]()
            {
                timer_callback();
                // std::cout << "Timer callback executed. count_ = " << message->data << std::endl;
                });
        }

        // void timer_callback()
        // {
        // if (message) 
        //     {
        //         message->data++;
        //         publisher_->publish(*message);
        //     } 
        // else
        //     {
        //         std::cout << "Error: message is null." << std::endl;
        //     }
        // }

        void timer_callback()
        {
            
            auto message = std_msgs::msg::Int32();
            message.data = count_;
            count_++;
            std::cout<<"msg.data: "<<message.data<<std::endl;
            publisher_->publish(message);
        }
        /*Destructor*/

        ~SimplePublisher()
        {
            std::cout << "Destructor called with value: "<< count_<< std::endl;
        }
private: 
        //std::shared_ptr<std_msgs::msg::Int32> message;
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}





