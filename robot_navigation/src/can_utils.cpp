/**
 * @file can_utils.cpp
 * @brief 公共 CAN 通信工具类实现
 */

#include "robot_navigation/can_utils.h"

#include <ros/ros.h>
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <poll.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

namespace robot_navigation {

// ── 构造/析构 ──────────────────────────────────────────────
CanInterface::CanInterface()
    : socket_fd_(-1)
{}

CanInterface::~CanInterface() {
  close();
}

// ── 打开 CAN 设备 ──────────────────────────────────────────
bool CanInterface::open(const std::string& device) {
  if (socket_fd_ >= 0) {
    // 已打开，如果是同一设备则复用，否则先关闭
    if (device == device_) return true;
    close();
  }

  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    ROS_ERROR("[can_utils] socket(PF_CAN) 失败: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", device.c_str());
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ROS_ERROR("[can_utils] ioctl(SIOCGIFINDEX) 失败(%s): %s",
              device.c_str(), std::strerror(errno));
    ::close(fd);
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR("[can_utils] bind(%s) 失败: %s", device.c_str(), std::strerror(errno));
    ::close(fd);
    return false;
  }

  socket_fd_ = fd;
  device_ = device;

  ROS_INFO("[can_utils] CAN 设备已连接: %s", device.c_str());
  return true;
}

// ── 关闭 CAN 设备 ──────────────────────────────────────────
void CanInterface::close() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
  device_.clear();
}

// ── 发送 CAN 帧 ────────────────────────────────────────────
bool CanInterface::sendFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;
  frame.can_dlc = (dlc > 8) ? 8 : dlc;
  if (data && dlc > 0) {
    std::memcpy(frame.data, data, frame.can_dlc);
  }
  return sendFrame(frame);
}

bool CanInterface::sendFrame(const struct can_frame& frame) {
  if (socket_fd_ < 0) {
    ROS_ERROR_THROTTLE(1.0, "[can_utils] CAN socket 未打开");
    return false;
  }

  const int nbytes = write(socket_fd_, &frame, sizeof(frame));
  if (nbytes != static_cast<int>(sizeof(frame))) {
    ROS_ERROR_THROTTLE(1.0, "[can_utils] CAN write 失败: %s", std::strerror(errno));
    return false;
  }
  return true;
}

// ── 接收 CAN 帧（阻塞，可超时） ─────────────────────────────
bool CanInterface::receiveFrame(struct can_frame& frame, int timeout_ms) {
  if (socket_fd_ < 0) return false;

  if (timeout_ms >= 0) {
    struct pollfd pfd;
    pfd.fd = socket_fd_;
    pfd.events = POLLIN;
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return false;
  }

  const int nbytes = read(socket_fd_, &frame, sizeof(frame));
  return (nbytes == static_cast<int>(sizeof(frame)));
}

// ── 校验和工具 ──────────────────────────────────────────────
uint8_t CanInterface::calcChecksumSum8(const uint8_t* data, uint8_t len) {
  if (!data || len == 0) return 0;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; ++i) {
    sum += data[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

uint8_t CanInterface::calcChecksumXor(const uint8_t* data, uint8_t len) {
  if (!data || len == 0) return 0;
  uint8_t x = 0;
  for (uint8_t i = 0; i < len; ++i) {
    x ^= data[i];
  }
  return x;
}

}  // namespace robot_navigation
