#ifndef ROS2_lidar__lidar_DRIVER_HPP_
#define ROS2_lidar__lidar_DRIVER_HPP_

#include <memory>
#include <map>
#include <string>

#include "ros2_lidar/conversions.hpp"

#include "ros2_lidar/interfaces/lifecycle_interface.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/empty.hpp"
#include "lidar_msgs/srv/get_metadata.hpp"

#include "tf2_ros/static_transform_broadcaster.h"

#include "ros2_lidar/interfaces/configuration.hpp"
#include "ros2_lidar/interfaces/data_processor_interface.hpp"

namespace ros2_lidar
{

class SensorInterface;

/**
 * @class ros2_lidar::lidarDriver
 * @brief A lifecycle interface implementation of a lidar OS-1 Lidar
 * driver in ROS2.
 */
class lidarDriver : public lifecycle_interface::LifecycleInterface
{
public:
  using DataProcessorMap = std::multimap<ClientState, DataProcessorInterface *>;
  using DataProcessorMapIt = DataProcessorMap::iterator;

  /**
   * @brief A constructor for ros2_lidar::lidarDriver
   * @param options Node options for lifecycle node interfaces
   */
  lidarDriver(
    std::unique_ptr<SensorInterface> sensor,
    const rclcpp::NodeOptions & options);

  /**
   * @brief A destructor for ros2_lidar::lidarDriver
   */
  ~lidarDriver();

  /**
   * @brief lifecycle node's implementation of configure step
   * which will configure ROS interfaces and allocate resources
   */
  void onConfigure() override;

  /**
   * @brief lifecycle node's implementation of activate step
   * which will activate ROS interfaces and start processing information
   */
  void onActivate() override;

  /**
   * @brief lifecycle node's implementation of deactivate step
   * which will deactivate ROS interfaces and stop processing information
   */
  void onDeactivate() override;

  /**
   * @brief lifecycle node's implementation of error step
   * which will handle errors in the lifecycle node system
   */
  void onError() override;

  /**
   * @brief lifecycle node's implementation of shutdown step
   * which will shut down the lifecycle node
   */
  void onShutdown() override;

  /**
   * @brief lifecycle node's implementation of cleanup step
   * which will deallocate resources
   */
  void onCleanup() override;

private:
  /**
  * @brief Timer callback to process the UDP socket
  */
  void processData();

  /**
   * @brief Create TF2 frames for the lidar sensor
   */
  void broadcastStaticTransforms(const ros2_lidar::Metadata & mdata);

  /**
  * @brief service callback to reset the lidar
  * @param request_header Header of rmw request
  * @param request Shared ptr of the Empty request
  * @param response Shared ptr of the Empty response
  */
  void resetService(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /**
  * @brief service callback to get metadata from lidar
  * @param request_header Header of rmw request
  * @param request Shared ptr of the GetMetadata request
  * @param response Shared ptr of the GetMetadata response
  */
  void getMetadata(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<lidar_msgs::srv::GetMetadata::Request> request,
    std::shared_ptr<lidar_msgs::srv::GetMetadata::Response> response);

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_srv;
  rclcpp::Service<lidar_msgs::srv::GetMetadata>::SharedPtr _metadata_srv;

  std::unique_ptr<SensorInterface> _sensor;
  std::multimap<ClientState, DataProcessorInterface *> _data_processors;
  rclcpp::TimerBase::SharedPtr _process_timer;

  std::string _laser_sensor_frame, _laser_data_frame, _imu_data_frame;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _tf_b;

  bool _use_system_default_qos;
  bool _use_ros_time;

  std::uint32_t _os1_proc_mask;
  ros2_lidar::Metadata mdata;
};

}  // namespace ros2_lidar

#endif  // ROS2_lidar__lidar_DRIVER_HPP_
