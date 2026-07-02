#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace robot_navigation {

/**
 * @brief 公共 CAN 通信工具类
 *
 * 封装 SocketCAN 的打开/关闭/发送/接收操作，
 * 供所有需要 CAN 通信的节点复用，避免代码重复。
 *
 * 线程安全：sendFrame 可多线程调用，接收需外部线程管理。
 *
 * 用法示例:
 *   CanInterface can;
 *   if (can.open("can1")) {
 *     can.sendFrame(0x206, data, 8);
 *   }
 */
class CanInterface {
 public:
  CanInterface();
  ~CanInterface();

  // ── 生命周期 ──
  bool open(const std::string& device);
  void close();
  bool isOpen() const { return socket_fd_ >= 0; }
  int  getFd() const { return socket_fd_; }

  // ── 通信 ──
  bool sendFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
  bool sendFrame(const struct can_frame& frame);

  // 阻塞接收（超时毫秒，-1=永久）
  bool receiveFrame(struct can_frame& frame, int timeout_ms = -1);

  // ── 工具 ──
  static uint8_t calcChecksumSum8(const uint8_t* data, uint8_t len);
  static uint8_t calcChecksumXor(const uint8_t* data, uint8_t len);

 private:
  int socket_fd_;
  std::string device_;
};

}  // namespace robot_navigation
