#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"


// ros2 launch ros_basic start_with_arguments.launch.py  msg_A:="Sith" msg_B:="Jedi" timer_period:=1.0


using namespace std::chrono_literals;

class DummyArgumetsExample : public rclcpp::Node {
public:
  DummyArgumetsExample(int &argc, char **argv) : Node("dummy_arguments_example") 
  {
      
    (void)argc; 
    // std::cout << "node: "<<argv[0]<<std::endl;
    // std::cout << "test: "<<argv[1]<<std::endl;

    message1 = argv[2];
    message2 = argv[3];
    timer_period = argv[5];
    
    float tp1 = std::stof(timer_period);
    auto tp2 = std::chrono::duration<double>(tp1);

    timer_ = this->create_wall_timer(
        tp2, std::bind(&DummyArgumetsExample::timer_callback, this));

    timer_flag_ = true;
  }

  void timer_callback()
   { 
    print_dummy_msgs(); 
   }

  void print_dummy_msgs() {

    if (timer_flag_) 
    {
      RCLCPP_INFO(this->get_logger(), "--- %s ---", message1.c_str());
      timer_flag_ = false;
    } 
    else 
    {
      RCLCPP_INFO(this->get_logger(), "--- %s ---", message2.c_str());
      timer_flag_ = true;
    }
  }

private:
  std::string timer_period;
  std::string message1;
  std::string message2;
  rclcpp::TimerBase::SharedPtr timer_;
  bool timer_flag_;
};

int main(int argc, char *argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyArgumetsExample>(argc, argv));
  rclcpp::shutdown();
  return 0;
}