#include <string>

#include "ros2_lidar/conversions.hpp"
#include "ros2_lidar/OS1/OS1_sensor.hpp"
#include "ros2_lidar/exception.hpp"
#include "ros2_lidar/interfaces/metadata.hpp"

namespace OS1
{

OS1Sensor::OS1Sensor()
: SensorInterface()
{
  // modified by zyl 	12608->1206  48->842

  _lidar_vendor = std::string("OLE_3D_V2");
  _lidar_packet_size = 1206;


  _lidar_packet.resize(_lidar_packet_size + 1);



}

OS1Sensor::~OS1Sensor()
{
  _lidar_client.reset();
  _lidar_packet.clear();

}

void OS1Sensor::reset(const ros2_lidar::Configuration & config)
{
  _lidar_client.reset();
  configure(config);
}

void OS1Sensor::configure(const ros2_lidar::Configuration & config)
{
  _lidar_vendor = 	config.lidar_vendor;
  _lidar_packet_size = 	(uint32_t)(config.lidar_packet_size);


  _lidar_packet.resize(_lidar_packet_size + 1);


  _lidar_client = OS1::init_client(config.lidar_port);

  if (!_lidar_client) {
    throw ros2_lidar::lidarDriverException(
            std::string("Failed to create connection to lidar."));
  }
}

ros2_lidar::ClientState OS1Sensor::get()
{
  const ros2_lidar::ClientState state = OS1::poll_client(*_lidar_client);

  if (state == ros2_lidar::ClientState::EXIT) {
    throw ros2_lidar::lidarDriverException(
            std::string(
              "Failed to get valid sensor data "
              "information from lidar, returned exit!"));
  } else if (state == ros2_lidar::ClientState::ERROR) {
    throw ros2_lidar::lidarDriverException(
            std::string(
              "Failed to get valid sensor data "
              "information from lidar, returned error!"));
  }

  return state;
}

uint8_t * OS1Sensor::readPacket(const ros2_lidar::ClientState & state)
{
  switch (state) {
    case ros2_lidar::ClientState::LIDAR_DATA:
      if (read_lidar_packet(*_lidar_client, _lidar_packet.data(),_lidar_packet_size)) {
        return _lidar_packet.data();
      } else {
        return nullptr;
      }
    default:
      return nullptr;
  }
}

}  // namespace OS1
