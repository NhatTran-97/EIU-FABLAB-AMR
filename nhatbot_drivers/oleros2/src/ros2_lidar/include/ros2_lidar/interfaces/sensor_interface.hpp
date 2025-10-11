#ifndef ROS2_lidar__INTERFACES__SENSOR_INTERFACE_HPP_
#define ROS2_lidar__INTERFACES__SENSOR_INTERFACE_HPP_

#include <memory>

#include "ros2_lidar/interfaces/metadata.hpp"
#include "ros2_lidar/interfaces/configuration.hpp"
#include "ros2_lidar/interfaces/data_processor_interface.hpp"

namespace ros2_lidar
{
/**
 * @class ros2_lidar::SensorInterface
 * @brief An interface for lidars units
 */
class SensorInterface
{
public:
  using SharedPtr = std::shared_ptr<SensorInterface>;
  using Ptr = std::unique_ptr<SensorInterface>;

  /**
   * @brief A sensor interface constructor
   */
  SensorInterface() {}

  /**
   * @brief A sensor interface destructor
   */
  virtual ~SensorInterface() = default;

  // copy
  SensorInterface(const SensorInterface &) = delete;
  SensorInterface & operator=(const SensorInterface &) = delete;

  // move
  SensorInterface(SensorInterface &&) = default;
  SensorInterface & operator=(SensorInterface &&) = default;

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   */
  virtual void reset(const ros2_lidar::Configuration & config) = 0;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   */
  virtual void configure(const ros2_lidar::Configuration & config) = 0;

  /**
   * @brief Ask sensor to get its current state for data collection
   * @return the state enum value
   */
  virtual ros2_lidar::ClientState get() = 0;

  /**
   * @brief reading the packet corresponding to the sensor state
   * @param state of the sensor
   * @return the packet of data
   */
  virtual uint8_t * readPacket(const ros2_lidar::ClientState & state) = 0;

  /**
   * @brief Get lidar sensor's metadata
   * @return sensor metadata struct
   */
  // virtual ros2_lidar::Metadata getMetadata() = 0;
};

}  // namespace ros2_lidar

#endif  // ROS2_lidar__INTERFACES__SENSOR_INTERFACE_HPP_
