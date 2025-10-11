// Copyright 2020, Steve Macenski
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_lidar__OS1__PROCESSORS__SCAN_PROCESSOR_HPP_
#define ROS2_lidar__OS1__PROCESSORS__SCAN_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/qos.hpp"

#include "ros2_lidar/conversions.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "ros2_lidar/interfaces/data_processor_interface.hpp"
#include "ros2_lidar/OS1/OS1.hpp"
#include "ros2_lidar/OS1/OS1_util.hpp"

namespace OS1
{
/**
 * @class OS1::ScanProcessor
 * @brief A data processor interface implementation of a processor
 * for creating Scans in the
 * driver in ROS2.
 */
class ScanProcessor : public ros2_lidar::DataProcessorInterface
{
public:
  using OSScan = std::vector<scan_os::ScanOS>;
  using OSScanIt = OSScan::iterator;

  /**
   * @brief A constructor for OS1::ScanProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  ScanProcessor(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const ros2_lidar::Metadata & mdata,
    const std::string & frame,
    const rclcpp::QoS & qos)
  : DataProcessorInterface(), _node(node), _frame(frame)
  {
    _pub = _node->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);

    _sin_lut = OS1::make_sin_lut();
    _cos_lut = OS1::make_cos_lut();
    _id_frame = 0;
    _id_col = 0;
    _azimuth_last = -1;
    _ts_last = -1;
    _height = mdata.num_lasers;
    _width = 6000;
    _realwidth = 2000;

    _aggregated_scans.resize(_width * _height);

    /*
    double zero_angle = 9999.0;
    _ring = 0;
    for (uint i = 0; i != _height; i++) {
      if (fabs(mdata.ah_offset_array[i]) < zero_angle) {
        _ring = static_cast<uint8_t>(i);
        zero_angle = fabs(mdata.ah_offset_array[i]);
      }
    }
    */
    _ring = mdata.ring_scan;
    if(mdata.lidar_vendor == std::string("OLE_2D_V2")){
      //std::cout << "scan_processor start" << std::endl;
      _batch_and_publish =
      OS1::batch_to_iter2<OSScanIt>(
      _sin_lut,
      _cos_lut,
      mdata.x_offset_array,
      mdata.y_offset_array,
      mdata.ah_offset_array,
      mdata.av_offset_array,
      _id_frame,
      _id_col,
      _azimuth_last,
      _ts_last,
	  _realwidth,
	  {},
	  &scan_os::ScanOS::make,
      [&](uint64_t scan_ts,uint32_t width) mutable
      {
        if (_pub->get_subscription_count() > 0 && _pub->is_activated()) {
          auto msg_ptr =
          std::make_unique<sensor_msgs::msg::LaserScan>(
            std::move(ros2_lidar::toMsg( _aggregated_scans, width, _node->get_clock()->now(), _frame, mdata, _ring)));
          _pub->publish(std::move(msg_ptr));
        }
      });  
    }
    else{
      _batch_and_publish =
      OS1::batch_to_iter3<OSScanIt>(
      _sin_lut,
      _cos_lut,
      mdata.x_offset_array,
      mdata.y_offset_array,
      mdata.ah_offset_array,
      mdata.av_offset_array,
      _id_frame,
      _id_col,
      _azimuth_last,
      _ts_last,
	  _realwidth,
	  {},
	  &scan_os::ScanOS::make,
      [&](uint64_t scan_ts,uint32_t width) mutable
      {
        if (_pub->get_subscription_count() > 0 && _pub->is_activated()) {
          auto msg_ptr =
          std::make_unique<sensor_msgs::msg::LaserScan>(
            std::move(ros2_lidar::toMsg( _aggregated_scans, width, _node->get_clock()->now(), _frame, mdata, _ring)));
          _pub->publish(std::move(msg_ptr));
        }
      });    
    }     
    
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~ScanProcessor()
  {
    _pub.reset();
  }

  /**
   * @brief Process method to create scan
   * @param data the packet data
   */
  bool process(uint8_t * data, uint64_t override_ts) override
  {
    OSScanIt it = _aggregated_scans.begin();
    _batch_and_publish(data, it, override_ts);
    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override
  {
    _pub->on_activate();
  }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override
  {
    _pub->on_deactivate();
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr _pub;
  std::function<void(const uint8_t *, OSScanIt, uint64_t)> _batch_and_publish;
  std::shared_ptr<pcl::PointCloud<scan_os::ScanOS>> _cloud;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;

  std::vector<double> _xyz_lut;
  OSScan _aggregated_scans;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;
  uint8_t _ring;

  //added by zyl
  std::vector<double> _cos_lut;
  std::vector<double> _sin_lut;
  uint64_t  _id_frame;          //serialNumber of frame
  uint32_t  _id_col;            //index of column
  int32_t   _azimuth_last;      //
  int64_t   _ts_last;           //timestamp of the 1st packet of last frame,
  uint32_t  _realwidth;
};

}  // namespace OS1

#endif  // ROS2_lidar__OS1__PROCESSORS__SCAN_PROCESSOR_HPP_
