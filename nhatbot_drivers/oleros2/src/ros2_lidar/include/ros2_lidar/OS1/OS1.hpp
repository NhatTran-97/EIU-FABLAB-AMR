#ifndef ROS2_lidar__OS1__OS1_HPP_
#define ROS2_lidar__OS1__OS1_HPP_

#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "arpa/inet.h"
#include "sys/socket.h"
#include "sys/types.h"

#include "jsoncpp/json/json.h"
#include "ros2_lidar/OS1/OS1_packet.hpp"
#include "ros2_lidar/interfaces/metadata.hpp"


namespace OS1
{

using ns = std::chrono::nanoseconds;

struct client
{
  ~client()
  {
    close(lidar_fd);

  }

  int lidar_fd{2368};  

  Json::Value meta;
};

const size_t lidar_packet_bytes = 1206;


/**
 * Connect to and configure the sensor and start listening for data
 * @param port port on which the sensor will receive lidar data
 * @return int of socket address
 */
inline int udp_data_socket(int port)
{
  struct addrinfo hints, * info_start, * ai;

  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET6;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_flags = AI_PASSIVE;

  auto port_s = std::to_string(port);

  int ret = getaddrinfo(NULL, port_s.c_str(), &hints, &info_start);
  if (ret != 0) {
    std::cerr << "getaddrinfo(): " << gai_strerror(ret) << std::endl;
    return -1;
  }
  if (info_start == NULL) {
    std::cerr << "getaddrinfo: empty result" << std::endl;
    return -1;
  }

  int sock_fd;
  for (ai = info_start; ai != NULL; ai = ai->ai_next) {
    sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
    if (sock_fd < 0) {
      std::cerr << "udp socket(): " << std::strerror(errno) << std::endl;
      continue;
    }

    if (bind(sock_fd, ai->ai_addr, ai->ai_addrlen) < 0) {
      close(sock_fd);
      std::cerr << "udp bind(): " << std::strerror(errno) << std::endl;
      continue;
    }

    break;
  }

  freeaddrinfo(info_start);
  if (ai == NULL) {
    close(sock_fd);
    return -1;
  }

  if (fcntl(sock_fd, F_SETFL, fcntl(sock_fd, F_GETFL, 0) | O_NONBLOCK) < 0) {
    std::cerr << "udp fcntl(): " << std::strerror(errno) << std::endl;
    return -1;
  }

  return sock_fd;
}

/**
 * Connect to and configure the sensor and start listening for data
 * @param lidar_port port on which the sensor will send lidar data

 * @return pointer owning the resources associated with the connection
 */
inline std::shared_ptr<client> init_client(int lidar_port = 2368)
{
  auto cli = std::make_shared<client>();

  int lidar_fd = udp_data_socket(lidar_port);
  
  cli->lidar_fd = lidar_fd;

  return cli;
}


/**
 * Block for up to timeout_sec until either data is ready or an error occurs.
 * @param cli client returned by init_client associated with the connection
 * @param timeout_sec seconds to block while waiting for data
 * @return ClientState s where (s & ERROR) is true if an error occured, (s &
 * LIDAR_DATA) is true if lidar data is ready to read
 */
inline ros2_lidar::ClientState poll_client(const client & c, const int timeout_sec = 1)
{
  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(c.lidar_fd, &rfds);


  timeval tv;
  tv.tv_sec = timeout_sec;
  tv.tv_usec = 0;

  int max_fd = c.lidar_fd;

  int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);

  if (retval == -1 && errno == EINTR) {
    return ros2_lidar::ClientState::EXIT;
  } else if (retval == -1) {
    std::cerr << "select: " << std::strerror(errno) << std::endl;
    return ros2_lidar::ClientState::ERROR;
  } else if (retval) {
    if (FD_ISSET(c.lidar_fd, &rfds)) {
      return ros2_lidar::ClientState::LIDAR_DATA;
    }
  }
  return ros2_lidar::ClientState::TIMEOUT;
}

/**
 * Read lidar data from the sensor. Will not block.
 * @param fd Socket connection for sensor data
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @param len length of packet
 * @return true if a lidar packet was successfully read
 */
static bool recv_fixed(int fd, void * buf, size_t len)
{
  ssize_t n = recvfrom(fd, buf, len + 1, 0, NULL, NULL);
  if (n == (ssize_t)len) {
    return true;
  } else if (n == -1) {
    std::cerr << "recvfrom: " << std::strerror(errno) << std::endl;
  } else {
    std::cerr << "Unexpected udp packet length: " << n << std::endl;
  }
  return false;
}

/**
 * Read lidar data from the sensor. Will not block.
 * @param cli client returned by init_client associated with the connection
 * @param buf buffer to which to write lidar data. Must be at least
 * lidar_packet_bytes + 1 bytes
 * @return true if a lidar packet was successfully read
 */
inline bool read_lidar_packet(const client & cli, uint8_t * buf, uint16_t packet_size)
{
  return recv_fixed(cli.lidar_fd, buf, packet_size);
}



}  // namespace OS1

#endif  // ROS2_lidar__OS1__OS1_HPP_
