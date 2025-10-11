#ifndef ROS2_lidar__OS1__OS1_SENSOR_HPP_
#define ROS2_lidar__OS1__OS1_SENSOR_HPP_

#include <memory>
#include <vector>

#include "ros2_lidar/OS1/processor_factories.hpp"

#include "ros2_lidar/interfaces/data_processor_interface.hpp"
#include "ros2_lidar/interfaces/sensor_interface.hpp"
#include "ros2_lidar/OS1/OS1.hpp"

namespace OS1
{

class OS1Sensor : public ros2_lidar::SensorInterface
{
public:
  OS1Sensor();

  ~OS1Sensor() override;

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   */
  void reset(const ros2_lidar::Configuration & config) override;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   */
  void configure(const ros2_lidar::Configuration & config) override;

  /**
   * @brief Get lidar sensor's metadata
   * @return sensor metadata struct
   */
  // ros2_lidar::Metadata getMetadata() override;

  /**
   * @brief Ask sensor to get its current state for data collection
   * @return the state enum value
   */
  ros2_lidar::ClientState get() override;

  /**
   * @brief reading the packet corresponding to the sensor state
   * @param state of the sensor
   * @return the packet of data
   */
  uint8_t * readPacket(const ros2_lidar::ClientState & state) override;

private:
  std::shared_ptr<client> _lidar_client;
  std::vector<uint8_t> _lidar_packet;
  // modified by zyl
  std::string _lidar_vendor;
  uint32_t _lidar_packet_size;

};

}  // namespace OS1

#endif  // ROS2_lidar__OS1__OS1_SENSOR_HPP_
