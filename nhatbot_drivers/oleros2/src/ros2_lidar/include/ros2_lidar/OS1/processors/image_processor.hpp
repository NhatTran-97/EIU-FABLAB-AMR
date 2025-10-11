#ifndef ROS2_lidar__OS1__PROCESSORS__IMAGE_PROCESSOR_HPP_
#define ROS2_lidar__OS1__PROCESSORS__IMAGE_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>
#include <algorithm>

#include <functional>

#include "rclcpp/qos.hpp"

#include "ros2_lidar/conversions.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "ros2_lidar/interfaces/data_processor_interface.hpp"
#include "ros2_lidar/OS1/OS1_util.hpp"

//#include "ros2_lidar/image_os.hpp"

namespace OS1
{
/**
 * @class OS1::ImageProcessor
 * @brief A data processor interface implementation of a processor
 * for creating range, intensity, and noise images in the
 * driver in ROS2.
 */
class ImageProcessor : public ros2_lidar::DataProcessorInterface
{
public:
  using OSImage = std::vector<picture_os::ImageOS>;
  using OSImageIt = OSImage::iterator;

  /**
   * @brief A constructor for OS1::ImageProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  ImageProcessor(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const ros2_lidar::Metadata & mdata,
    const std::string & frame,
    const rclcpp::QoS & qos)
  : DataProcessorInterface(), _node(node), _frame(frame)
  {
	 int px_init[]={0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18, 0, 6, 12, 18};
	 _px_offset.reserve(16);
	 _px_offset.assign(&px_init[0],&px_init[16]);

	 _sin_lut = OS1::make_sin_lut();
	 _cos_lut = OS1::make_cos_lut();
	 _id_frame = 0;
	 _id_col = 0;
	 _azimuth_last = -1;
	 _ts_last = -1;
	 _height = mdata.num_lasers;
	 _width = 2000;
	 _realwidth = 2000;

    _range_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "range_image", qos);
    _intensity_image_pub = _node->create_publisher<sensor_msgs::msg::Image>(
      "intensity_image", qos);

    _range_image.width = _width;
    _range_image.height = _height;
    _range_image.step = _width;
    _range_image.encoding = "mono8";
    _range_image.header.frame_id = _frame;
    _range_image.data.resize(_width * _height);

    _intensity_image.width = _width;
    _intensity_image.height = _height;
    _intensity_image.step = _width;
    _intensity_image.encoding = "mono8";
    _intensity_image.header.frame_id = _frame;
    _intensity_image.data.resize(_width * _height);


    _information_image.resize(_width * _height);

    if(mdata.lidar_vendor == std::string("OLE_2D_V2")){
    std::cout << "image_processor start" << std::endl;
      _batch_and_publish = OS1::batch_to_iter2<OSImageIt>(
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
	  &picture_os::ImageOS::make,
      [&](uint64_t scan_ts,uint32_t width) mutable
      {
        rclcpp::Time t(scan_ts);
        _range_image.header.stamp = t;
        _intensity_image.header.stamp = t;

        OSImageIt it;
        for (uint u = 0; u != _height; u++) {
          for (uint v = 0; v != width; v++) {
            const size_t vv = (v + _px_offset[u]) % width;
            const size_t index = vv * _height + u;
            picture_os::ImageOS & px = _information_image[index];

            const uint & idx = u * width + v;
            if (px.range == 0) {
              _range_image.data[idx] = 0;
            } else {
              _range_image.data[idx] = 255 - std::min(std::round((float)(px.range * 1e-3)), 255.0f);
            }

            _intensity_image.data[idx] = std::min(px.intensity, 255.0f);
          }
        }

        if (_range_image_pub->get_subscription_count() > 0 &&
        _range_image_pub->is_activated())
        {
          _range_image_pub->publish(_range_image);
        }

        if (_intensity_image_pub->get_subscription_count() > 0 &&
        _intensity_image_pub->is_activated())
        {
          _intensity_image_pub->publish(_intensity_image);
        }

      }); 
    }
    else{
      _batch_and_publish =
      OS1::batch_to_iter3<OSImageIt>(
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
	  &picture_os::ImageOS::make,
      [&](uint64_t scan_ts,uint32_t width) mutable
      {
        rclcpp::Time t(scan_ts);
        _range_image.header.stamp = t;
        _intensity_image.header.stamp = t;

        OSImageIt it;
        for (uint u = 0; u != _height; u++) {
          for (uint v = 0; v != width; v++) {
            const size_t vv = (v + _px_offset[u]) % width;
            const size_t index = vv * _height + u;
            picture_os::ImageOS & px = _information_image[index];

            const uint & idx = u * width + v;
            if (px.range == 0) {
              _range_image.data[idx] = 0;
            } else {
              _range_image.data[idx] = 255 - std::min(std::round((float)(px.range * 1e-3)), 255.0f);
            }

            _intensity_image.data[idx] = std::min(px.intensity, 255.0f);
          }
        }

        if (_range_image_pub->get_subscription_count() > 0 &&
        _range_image_pub->is_activated())
        {
          _range_image_pub->publish(_range_image);
        }

        if (_intensity_image_pub->get_subscription_count() > 0 &&
        _intensity_image_pub->is_activated())
        {
          _intensity_image_pub->publish(_intensity_image);
        }

      });  
    }
    
  }

  /**
   * @brief A destructor clearing memory allocated
   */
  ~ImageProcessor()
  {
    _intensity_image_pub.reset();
    _range_image_pub.reset();
  }

  /**
   * @brief Process method to create images
   * @param data the packet data
   */
  bool process(uint8_t * data, uint64_t override_ts) override
  {
    OSImageIt it = _information_image.begin();
    _batch_and_publish(data, it, override_ts);
    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override
  {
    _intensity_image_pub->on_activate();
    _range_image_pub->on_activate();
  }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override
  {
    _intensity_image_pub->on_deactivate();
    _range_image_pub->on_deactivate();
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _intensity_image_pub;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr _range_image_pub;
  std::function<void(const uint8_t *, OSImageIt, uint64_t)> _batch_and_publish;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  sensor_msgs::msg::Image _intensity_image;
  sensor_msgs::msg::Image _range_image;

  std::vector<double> _xyz_lut;
  std::vector<int> _px_offset;
  OSImage _information_image;
  std::string _frame;
  uint32_t _height;
  uint32_t _width;

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

#endif  // ROS2_lidar__OS1__PROCESSORS__IMAGE_PROCESSOR_HPP_
