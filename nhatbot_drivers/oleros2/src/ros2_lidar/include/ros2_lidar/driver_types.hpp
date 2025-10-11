#ifndef ROS2_lidar__DRIVER_TYPES_HPP_
#define ROS2_lidar__DRIVER_TYPES_HPP_

#include "ros2_lidar/lidar_driver.hpp"

namespace ros2_lidar
{

class OS1Driver : public lidarDriver
{
public:
  explicit OS1Driver(rclcpp::NodeOptions options);
};

}  // namespace ros2_lidar


#endif  // ROS2_lidar__DRIVER_TYPES_HPP_
