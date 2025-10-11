#ifndef ROS2_lidar__EXCEPTION_HPP_
#define ROS2_lidar__EXCEPTION_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace ros2_lidar
{

/**
 * @class lidarDriverException
 * @brief Thrown when lidar lidar driver encounters a fatal error
 */
class lidarDriverException : public std::runtime_error
{
public:
  /**
   * @brief A constructor for ros2_lidar::lidarDriverException
   * @param description string to display the exception message
   */
  explicit lidarDriverException(const std::string description)
  : std::runtime_error(description) {}
};

}  // namespace ros2_lidar

#endif  // ROS2_lidar__EXCEPTION_HPP_
