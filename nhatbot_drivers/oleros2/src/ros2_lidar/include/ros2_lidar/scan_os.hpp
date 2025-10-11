#ifndef ROS2_lidar__SCAN_OS_HPP_
#define ROS2_lidar__SCAN_OS_HPP_

#define PCL_NO_PRECOMPILE
#include "pcl/point_types.h"

namespace scan_os
{

/**
 * @struct scan_os::ScanOS
 * @brief The Point template for PCL to hold the information for
 * publishing in ROS2.
 */
struct EIGEN_ALIGN16 ScanOS
{
  float intensity;
  uint32_t range;
  uint32_t t;
  uint8_t ring;
  float frequency;

  /**
   * @brief A factory to create the structure given a set of information
   * @param x position in m
   * @param y position in m
   * @param z position in m
   * @param intensity Intensity value from reading
   * @param t time in ns
   * @param reflectivity Reflectivity value from reading
   * @param ring The ring ID the measurement came from
   * @param column The column of the reading in the ring
   * @param frequency value from reading
   * @param range Range value from reading
   */
  static inline ScanOS make(
    float x, float y, float z, float intensity,
    uint32_t t, uint16_t reflectivity, uint8_t ring, uint8_t col,
    float frequency, uint32_t range)
  {
    return {intensity, range, t,ring,frequency};
  }
};

}  // namespace scan_os

#endif  // ROS2_lidar__SCAN_OS_HPP_
