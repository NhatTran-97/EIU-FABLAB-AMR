#ifndef ROS2_lidar__INTERFACES__METADATA_HPP_
#define ROS2_lidar__INTERFACES__METADATA_HPP_

#include <vector>
#include <string>

namespace ros2_lidar
{

/**
 * @brief client response on current state
 */
enum ClientState
{
  TIMEOUT = 0,
  ERROR = 1,
  LIDAR_DATA = 2,
  IMU_DATA = 4,
  EXIT = 8
};

/**
 * @brief metadata about lidar lidar sensor
 */
struct Metadata
{
  std::string computer_ip;
  std::string lidar_ip;
  int lidar_port;

  std::vector<double> lidar_to_sensor_transform;

  // added by zyl
  std::vector<double> x_offset_array;
  std::vector<double> y_offset_array;
  std::vector<double> ah_offset_array;
  std::vector<double> av_offset_array;
  std::vector<int64_t> laser_id_array;
  int num_lasers;
  double distance_resolution;
  int ring_scan;
  std::string lidar_vendor;
  int lidar_packet_size;
};

}  // namespace ros2_lidar

#endif  // ROS2_lidar__INTERFACES__METADATA_HPP_
