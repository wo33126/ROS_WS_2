/**
 * @file display_node.cpp
 * @brief 机器人显示屏节点
 *
 * 订阅 /robot_display 话题 (std_msgs/String)，将文本显示到：
 *   1. ROS 日志（主要输出途径）
 *   2. 可选的 CAN 发送给 STM32 液晶屏
 *
 * 条形码数值在比赛期间一直显示。
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_navigation {

class DisplayNode {
 public:
  DisplayNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , can_enable_(false)
      , can_device_("can1")
      , can_id_(0x300)
      , last_text_("")
  {
    pnh_.param<bool>("can_enable", can_enable_, false);
    pnh_.param<std::string>("can_device", can_device_, "can1");
    int can_id_int = static_cast<int>(can_id_);
    pnh_.param("can_id", can_id_int, can_id_int);
    can_id_ = static_cast<uint32_t>(can_id_int);
  }

  bool init() {
    sub_ = nh_.subscribe("/robot_display", 1, &DisplayNode::displayCallback, this);

    ROS_INFO("[display_node] 初始化完成");
    ROS_INFO("[display_node]   订阅: /robot_display");
    if (can_enable_) {
      ROS_INFO("[display_node]   CAN输出: %s, ID=0x%03X", can_device_.c_str(), can_id_);
    } else {
      ROS_INFO("[display_node]   输出模式: ROS日志");
    }

    return true;
  }

 private:
  void displayCallback(const std_msgs::String::ConstPtr& msg) {
    if (msg->data == last_text_) return;
    last_text_ = msg->data;

    // ── 输出到 ROS 日志 ──
    ROS_INFO("[DISPLAY] %s", msg->data.c_str());

    // ── 可选: 通过 CAN 发送到 STM32 液晶屏 ──
    if (can_enable_) {
      sendCanDisplay(msg->data);
    }
  }

  void sendCanDisplay(const std::string& text) {
    // CAN 液晶屏显示协议（自定义）：
    // 将文本按 8 字节分帧发送
    // 每帧: [0]=0x30(显示指令), [1-7]=ASCII字符
    // 由于单帧仅能容纳 7 字节，长文本需分包或仅发送摘要

    static int socket_fd = -1;

    if (socket_fd < 0) {
      socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
      if (socket_fd < 0) return;

      struct ifreq ifr;
      std::memset(&ifr, 0, sizeof(ifr));
      std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_device_.c_str());
      if (ioctl(socket_fd, SIOCGIFINDEX, &ifr) < 0) {
        close(socket_fd);
        socket_fd = -1;
        return;
      }

      struct sockaddr_can addr;
      std::memset(&addr, 0, sizeof(addr));
      addr.can_family = AF_CAN;
      addr.can_ifindex = ifr.ifr_ifindex;
      if (bind(socket_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(socket_fd);
        socket_fd = -1;
        return;
      }
    }

    // 发送第一帧（前7字节 + 指令码）
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id_;
    frame.can_dlc = 8;
    frame.data[0] = 0x30;  // 显示指令码
    for (size_t i = 0; i < 7 && i < text.length(); ++i) {
      frame.data[i + 1] = static_cast<uint8_t>(text[i]);
    }

    write(socket_fd, &frame, sizeof(frame));

    // 如有更多文本，发送第二帧
    if (text.length() > 7) {
      std::memset(&frame, 0, sizeof(frame));
      frame.can_id = can_id_;
      frame.can_dlc = 8;
      frame.data[0] = 0x31;  // 续帧指令码
      for (size_t i = 7; i < 14 && i < text.length(); ++i) {
        frame.data[i - 6] = static_cast<uint8_t>(text[i]);
      }
      write(socket_fd, &frame, sizeof(frame));
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_;

  bool can_enable_;
  std::string can_device_;
  uint32_t can_id_;
  std::string last_text_;
};

}  // namespace robot_navigation

int main(int argc, char** argv) {
  ros::init(argc, argv, "display_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::DisplayNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[display_node] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
