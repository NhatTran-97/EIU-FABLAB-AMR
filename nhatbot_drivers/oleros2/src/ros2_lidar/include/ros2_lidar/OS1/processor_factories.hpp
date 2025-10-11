#ifndef ROS2_lidar__OS1__PROCESSOR_FACTORIES_HPP_
#define ROS2_lidar__OS1__PROCESSOR_FACTORIES_HPP_

#include <cstdint>
#include <string>
#include <map>
#include <utility>

#include "rclcpp/qos.hpp"
#include "ros2_lidar/conversions.hpp"
#include "ros2_lidar/string_utils.hpp"
#include "ros2_lidar/OS1/processors/image_processor.hpp"
#include "ros2_lidar/OS1/processors/pointcloud_processor.hpp"
#include "ros2_lidar/OS1/processors/scan_processor.hpp"

namespace ros2_lidar
{

constexpr std::uint32_t OS1_PROC_IMG = (1 << 0);
constexpr std::uint32_t OS1_PROC_PCL = (1 << 1);
constexpr std::uint32_t OS1_PROC_SCAN = (1 << 3);

constexpr std::uint32_t OS1_DEFAULT_PROC_MASK =
  OS1_PROC_IMG | OS1_PROC_PCL | OS1_PROC_SCAN;

/**
 * Transforms a data processor mask-like-string into a mask value
 *
 * We define a mask-like-string to be a pipe-separated list of
 * data processor suffixes. For example, all of the following
 * are valid:
 
 * IMG|PCL
 * PCL
 *
 * @param[in] mask_str The string to convert into a mask
 * @return The mask obtained from the parsed input string.
 */
inline std::uint32_t
toProcMask(const std::string & mask_str)
{
  std::uint32_t mask = 0x0;
  auto tokens = ros2_lidar::split(mask_str, '|');

  for (auto & token : tokens) {
    if (token == "IMG") {
      mask |= ros2_lidar::OS1_PROC_IMG;
    } else if (token == "PCL") {
      mask |= ros2_lidar::OS1_PROC_PCL;
    } else if (token == "SCAN") {
      mask |= ros2_lidar::OS1_PROC_SCAN;
    }
  }

  return mask;
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the image (range, intensity, noise) interfaces
 * @return Raw pointer to a data processor interface to use
 */
inline ros2_lidar::DataProcessorInterface * createImageProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_lidar::Metadata & mdata,
  const std::string & frame,
  const rclcpp::QoS & qos)
{
  return new OS1::ImageProcessor(node, mdata, frame, qos);
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the pointcloud (PointXYZ be default) interface
 * @return Raw pointer to a data processor interface to use
 */
inline ros2_lidar::DataProcessorInterface * createPointcloudProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_lidar::Metadata & mdata,
  const std::string & frame,
  const rclcpp::QoS & qos)
{
  return new OS1::PointcloudProcessor(node, mdata, frame, qos);
}




/**
 * @brief Factory method to get a pointer to a processor
 * to create the scan interfaces
 * @return Raw pointer to a data processor interface to use
 */
inline ros2_lidar::DataProcessorInterface * createScanProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_lidar::Metadata & mdata,
  const std::string & frame,
  const rclcpp::QoS & qos)
{
  return new OS1::ScanProcessor(node, mdata, frame, qos);
}

inline std::multimap<ClientState, DataProcessorInterface *> createProcessors(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const ros2_lidar::Metadata & mdata,
  const std::string & laser_frame,
  const rclcpp::QoS & qos,
  std::uint32_t mask = ros2_lidar::OS1_DEFAULT_PROC_MASK)
{
  std::multimap<ClientState, DataProcessorInterface *> data_processors;

  if ((mask & ros2_lidar::OS1_PROC_IMG) == ros2_lidar::OS1_PROC_IMG) {
    data_processors.insert(
      std::pair<ClientState, DataProcessorInterface *>(
        ClientState::LIDAR_DATA, createImageProcessor(
          node, mdata, laser_frame, qos)));
  }

  if ((mask & ros2_lidar::OS1_PROC_PCL) == ros2_lidar::OS1_PROC_PCL) {
    data_processors.insert(
      std::pair<ClientState, DataProcessorInterface *>(
        ClientState::LIDAR_DATA, createPointcloudProcessor(
          node, mdata, laser_frame, qos)));
  }


  if ((mask & ros2_lidar::OS1_PROC_SCAN) == ros2_lidar::OS1_PROC_SCAN) {
    data_processors.insert(
      std::pair<ClientState, DataProcessorInterface *>(
        ClientState::LIDAR_DATA, createScanProcessor(
          node, mdata, laser_frame, qos)));
  }

  return data_processors;
}

}  // namespace ros2_lidar

#endif  // ROS2_lidar__OS1__PROCESSOR_FACTORIES_HPP_
