#ifndef ROS2_lidar__OS1__OS1_PACKET_HPP_
#define ROS2_lidar__OS1__OS1_PACKET_HPP_

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace OS1
{

const int pixels_per_column = 64;
const int columns_per_buffer = 16;

const int pixel_bytes = 12;
const int column_bytes = 16 + (pixels_per_column * pixel_bytes) + 4;

}  // namespace OS1

#endif  // ROS2_lidar__OS1__OS1_PACKET_HPP_
