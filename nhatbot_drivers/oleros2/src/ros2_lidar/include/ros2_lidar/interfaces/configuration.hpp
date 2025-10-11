#ifndef ROS2_lidar__INTERFACES__CONFIGURATION_HPP_
#define ROS2_lidar__INTERFACES__CONFIGURATION_HPP_

#include <string>

namespace ros2_lidar
{
/**
 * @brief Configuration parameters to send to lidar lidar
 */
struct Configuration
{
  std::string lidar_ip;
  std::string computer_ip;
  int lidar_port;
  std::string lidar_mode;
  std::string timestamp_mode;

  std::string lidar_vendor;
  int lidar_packet_size;

};

}  // namespace ros2_lidar

#endif  // ROS2_lidar__INTERFACES__CONFIGURATION_HPP_
