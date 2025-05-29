#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// To use 3s values, for example,
using namespace std::chrono_literals;

int32_t main(const int32_t argc, char **const argv) 
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("wait_set_listener");
  auto do_nothing = [](std_msgs::msg::String::UniquePtr) 
  { 
    assert(false); 
  };

  auto sub1 = node->create_subscription<std_msgs::msg::String>(
      "/reached_goal/box_bot_1", 1, do_nothing);
  auto sub2 = node->create_subscription<std_msgs::msg::String>(
      "/reached_goal/box_bot_2", 1, do_nothing);
  auto sub3 = node->create_subscription<std_msgs::msg::String>(
      "/reached_goal/box_bot_3", 1, do_nothing);

  rclcpp::StaticWaitSet<3, 0, 0, 0, 0, 0> wait_set(
      std::array<rclcpp::StaticWaitSet<3, 0, 0, 0, 0, 0>::SubscriptionEntry, 3>
      {
          {
            {sub1}, {sub2}, {sub3}}
          },
      std::array<rclcpp::GuardCondition::SharedPtr, 0>{},
      std::array<rclcpp::TimerBase::SharedPtr, 0>{},
      std::array<rclcpp::ClientBase::SharedPtr, 0>{},
      std::array<rclcpp::ServiceBase::SharedPtr, 0>{},
      std::array<rclcpp::StaticWaitSet<3, 0, 0, 0, 0, 0>::WaitableEntry, 0>{});

  while (rclcpp::ok()) 
  {
    const auto wait_result = wait_set.wait(3s);
    if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
      bool sub1_has_data =
          wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0U];
      bool sub2_has_data =
          wait_result.get_wait_set().get_rcl_wait_set().subscriptions[1U];
      bool sub3_has_data =
          wait_result.get_wait_set().get_rcl_wait_set().subscriptions[2U];

      if (sub1_has_data) 
      {
        std_msgs::msg::String topic_msg;
        rclcpp::MessageInfo msg_info;

        sub1->take(topic_msg, msg_info);
        RCLCPP_INFO(node->get_logger(), "I heard: '%s'",
                    topic_msg.data.c_str());
      }

      if (sub2_has_data) 
      {
        std_msgs::msg::String topic_msg;
        rclcpp::MessageInfo msg_info;

        sub2->take(topic_msg, msg_info);
        RCLCPP_INFO(node->get_logger(), "I heard: '%s'",
                    topic_msg.data.c_str());
      }

      if (sub3_has_data) 
      {
        std_msgs::msg::String topic_msg;
        rclcpp::MessageInfo msg_info;

        sub3->take(topic_msg, msg_info);
        RCLCPP_INFO(node->get_logger(), "I heard: '%s'",
                    topic_msg.data.c_str());
      }

    } 
    else if (wait_result.kind() == rclcpp::WaitResultKind::Timeout) 
    {
      if (rclcpp::ok()) 
      {
        RCLCPP_ERROR(node->get_logger(), "Wait-set failed with timeout");
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}