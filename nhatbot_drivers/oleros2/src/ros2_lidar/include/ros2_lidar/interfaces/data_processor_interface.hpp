#ifndef ROS2_lidar__INTERFACES__DATA_PROCESSOR_INTERFACE_HPP_
#define ROS2_lidar__INTERFACES__DATA_PROCESSOR_INTERFACE_HPP_

#include <memory>
#include <string>

#include "ros2_lidar/interfaces/metadata.hpp"
#include "ros2_lidar/interfaces/configuration.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_lidar
{
/**
 * @class ros2_lidar::DataProcessorInterface
 * @brief An interface for data processors using a
 * lidar-specific API to create structured information
 * like pointclouds, images, or directly publishing
 * packets or IMU data.
 */
class DataProcessorInterface
{
public:
  /**
   * @brief Constructor of the data processor interface
   */
  DataProcessorInterface() {}

  /**
   * @brief Destructor of the data processor interface
   */
  virtual ~DataProcessorInterface() = default;

  /**
   * @brief Process a packet with the lidar-specific APIs
   * @param data packet input
   * @param override_ts Timestamp in nanos to use to override the ts in the
   *                    packet data. To use the packet data, pass as 0.
   */
  virtual bool process(uint8_t * data, uint64_t override_ts = 0) = 0;

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  virtual void onActivate() = 0;

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  virtual void onDeactivate() = 0;
};

}  // namespace ros2_lidar

#endif  // ROS2_lidar__INTERFACES__DATA_PROCESSOR_INTERFACE_HPP_
