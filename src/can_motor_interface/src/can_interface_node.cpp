#include "can_motor_interface/can_interface.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace can_motor_interface {

CanInterfaceNode::CanInterfaceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      socket_fd_(-1),
      running_(true),
      tx_can_id_(0x201),
      rx_can_id_(0x181),
      rx_can_id_filter_enable_(true),
      use_extended_frame_(true),
      payload_little_endian_(true),
      checksum_use_sum8_(true),
      speed_cmd_code_(0x01),
      status_cmd_code_(0x81),
      broadcast_index_(0xFF),
      max_rpm_(3000.0f),
      motor_count_(4),
      heartbeat_timeout_sec_(1.0) {
  pnh_.param<std::string>("can_device", can_device_, "can0");

  int tx_can_id_i = static_cast<int>(tx_can_id_);
  int rx_can_id_i = static_cast<int>(rx_can_id_);
  pnh_.param("tx_can_id", tx_can_id_i, tx_can_id_i);
  pnh_.param("rx_can_id", rx_can_id_i, rx_can_id_i);
  tx_can_id_ = static_cast<uint32_t>(tx_can_id_i);
  rx_can_id_ = static_cast<uint32_t>(rx_can_id_i);
  pnh_.param("rx_can_id_filter_enable", rx_can_id_filter_enable_, rx_can_id_filter_enable_);
  pnh_.param("use_extended_frame", use_extended_frame_, use_extended_frame_);

  pnh_.param("payload_little_endian", payload_little_endian_, payload_little_endian_);
  pnh_.param("checksum_use_sum8", checksum_use_sum8_, checksum_use_sum8_);

  int speed_cmd_code_i = static_cast<int>(speed_cmd_code_);
  pnh_.param("speed_cmd_code", speed_cmd_code_i, speed_cmd_code_i);
  speed_cmd_code_ = static_cast<uint8_t>(speed_cmd_code_i & 0xFF);

  int status_cmd_code_i = static_cast<int>(status_cmd_code_);
  pnh_.param("status_cmd_code", status_cmd_code_i, status_cmd_code_i);
  status_cmd_code_ = static_cast<uint8_t>(status_cmd_code_i & 0xFF);

  int broadcast_index_i = static_cast<int>(broadcast_index_);
  pnh_.param("broadcast_index", broadcast_index_i, broadcast_index_i);
  broadcast_index_ = static_cast<uint8_t>(broadcast_index_i & 0xFF);

  nh_.param("/robot/max_rpm", max_rpm_, max_rpm_);
  pnh_.param("max_rpm", max_rpm_, max_rpm_);
  pnh_.param("motor_count", motor_count_, motor_count_);
  pnh_.param("heartbeat_timeout_sec", heartbeat_timeout_sec_, heartbeat_timeout_sec_);

  motor_count_ = std::max(1, std::min(255, motor_count_));
  motor_ids_.clear();
  motor_ids_.reserve(static_cast<size_t>(motor_count_));
  for (int i = 0; i < motor_count_; ++i) {
    motor_ids_.push_back(i);
  }

  motor_state_rpm_.assign(motor_ids_.size(), 0.0f);
  motor_status_flags_.assign(motor_ids_.size(), 0);

  cmd_sub_ = nh_.subscribe("/motor_velocity_cmd", 20, &CanInterfaceNode::cmdCallback, this);
  motor_state_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/motor_state", 20);
  motor_status_flag_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/motor_status_flags", 20);
  emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 10, true);
  can_rx_pub_ = nh_.advertise<can_msgs::Frame>("/can_rx", 50);
  monitor_timer_ = nh_.createTimer(ros::Duration(0.2), &CanInterfaceNode::timerCallback, this);

  if (!openSocket()) {
    ROS_WARN("CAN socket open failed at startup, will retry in timer.");
  }

  last_rx_time_ = ros::Time::now();
  rx_thread_ = std::thread(&CanInterfaceNode::canReceiveThread, this);
}

CanInterfaceNode::~CanInterfaceNode() {
  running_.store(false);
  closeSocket();
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

bool CanInterfaceNode::openSocket() {
  if (socket_fd_ >= 0) {
    return true;
  }

  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    ROS_ERROR("socket(PF_CAN, SOCK_RAW) failed: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_device_.c_str());
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ROS_ERROR("ioctl(SIOCGIFINDEX) failed for %s: %s", can_device_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR("bind() failed on %s: %s", can_device_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  socket_fd_ = fd;
  ROS_INFO("SocketCAN connected on %s", can_device_.c_str());
  return true;
}

void CanInterfaceNode::closeSocket() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool CanInterfaceNode::sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc) {
  if (socket_fd_ < 0 && !openSocket()) {
    return false;
  }

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = can_id;
  const uint8_t safe_dlc = std::min<uint8_t>(dlc, 8);
  frame.can_dlc = safe_dlc;
  std::memcpy(frame.data, data, safe_dlc);

  const int nbytes = write(socket_fd_, &frame, sizeof(frame));
  if (nbytes != static_cast<int>(sizeof(frame))) {
    ROS_ERROR_THROTTLE(1.0, "CAN write failed: %s", std::strerror(errno));
    closeSocket();
    return false;
  }
  return true;
}

uint8_t CanInterfaceNode::calcChecksum(const uint8_t* data, uint8_t len) const {
  if (!data || len == 0) {
    return 0;
  }

  if (checksum_use_sum8_) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; ++i) {
      sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
  }

  uint8_t x = 0;
  for (uint8_t i = 0; i < len; ++i) {
    x ^= data[i];
  }
  return x;
}

bool CanInterfaceNode::sendSpeedCommand(uint8_t motor_index, float target_rpm) {
  const float rpm = std::max(-max_rpm_, std::min(max_rpm_, target_rpm));
  const int32_t rpm_i = static_cast<int32_t>(std::lround(rpm));

  uint8_t data[7] = {0};
  data[0] = speed_cmd_code_;  // 0x01: 速度控制
  data[1] = motor_index;      // 0~3 或 0xFF（广播）

  if (payload_little_endian_) {
    data[2] = static_cast<uint8_t>(rpm_i & 0xFF);
    data[3] = static_cast<uint8_t>((rpm_i >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>((rpm_i >> 16) & 0xFF);
    data[5] = static_cast<uint8_t>((rpm_i >> 24) & 0xFF);
  } else {
    data[2] = static_cast<uint8_t>((rpm_i >> 24) & 0xFF);
    data[3] = static_cast<uint8_t>((rpm_i >> 16) & 0xFF);
    data[4] = static_cast<uint8_t>((rpm_i >> 8) & 0xFF);
    data[5] = static_cast<uint8_t>(rpm_i & 0xFF);
  }

  data[6] = calcChecksum(data, 6);

  uint32_t can_id = tx_can_id_;
  if (use_extended_frame_) {
    can_id = (can_id & CAN_EFF_MASK) | CAN_EFF_FLAG;
  } else {
    can_id &= CAN_SFF_MASK;
  }
  return sendCanFrame(can_id, data, 7);
}

void CanInterfaceNode::cmdCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if (msg->data.empty()) {
    ROS_WARN_THROTTLE(1.0, "Received empty /motor_velocity_cmd.");
    return;
  }

  const size_t n = std::min(msg->data.size(), static_cast<size_t>(motor_count_));
  if (msg->data.size() != static_cast<size_t>(motor_count_)) {
    ROS_WARN_THROTTLE(1.0,
                      "Expected /motor_velocity_cmd length=%d, got %zu, using first %zu values.",
                      motor_count_, msg->data.size(), n);
  }

  for (size_t i = 0; i < n; ++i) {
    sendSpeedCommand(static_cast<uint8_t>(i), msg->data[i]);
  }

  for (size_t i = n; i < static_cast<size_t>(motor_count_); ++i) {
    sendSpeedCommand(static_cast<uint8_t>(i), 0.0f);
  }
}

void CanInterfaceNode::parseStatusFrame(const can_msgs::Frame& frame) {
  if (frame.dlc < 2) {
    return;
  }

  if (rx_can_id_filter_enable_) {
    const uint32_t rx_id = frame.is_extended ? (frame.id & CAN_EFF_MASK) : (frame.id & CAN_SFF_MASK);
    const uint32_t cfg_id = use_extended_frame_ ? (rx_can_id_ & CAN_EFF_MASK) : (rx_can_id_ & CAN_SFF_MASK);
    if (rx_id != cfg_id) {
      return;
    }
  }

  if (frame.data[0] != status_cmd_code_) {
    return;
  }

  // 约定状态帧格式：
  // data[0] 命令码（默认 0x81）
  // data[1] 电机索引（0~3 或 0xFF）
  // data[2] 状态标志（可选）
  // data[3..6] int32 实际转速 RPM（可选）
  // data[last] 校验
  const uint8_t last = static_cast<uint8_t>(frame.dlc - 1);
  if (calcChecksum(frame.data.data(), last) != frame.data[last]) {
    ROS_WARN_THROTTLE(1.0, "Drop STM32 state frame due to checksum mismatch.");
    return;
  }

  const uint8_t motor_index = frame.data[1];
  if (motor_index != broadcast_index_ && motor_index >= static_cast<uint8_t>(motor_count_)) {
    return;
  }

  auto decodeI32 = [&](uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) -> int32_t {
    uint32_t u = 0;
    if (payload_little_endian_) {
      u = static_cast<uint32_t>(b0) |
          (static_cast<uint32_t>(b1) << 8) |
          (static_cast<uint32_t>(b2) << 16) |
          (static_cast<uint32_t>(b3) << 24);
    } else {
      u = (static_cast<uint32_t>(b0) << 24) |
          (static_cast<uint32_t>(b1) << 16) |
          (static_cast<uint32_t>(b2) << 8) |
          static_cast<uint32_t>(b3);
    }
    return static_cast<int32_t>(u);
  };

  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    if (frame.dlc >= 8) {
      const uint8_t flags = frame.data[2];
      const int32_t rpm = decodeI32(frame.data[3], frame.data[4], frame.data[5], frame.data[6]);

      if (motor_index == broadcast_index_) {
        for (int i = 0; i < motor_count_; ++i) {
          motor_state_rpm_[static_cast<size_t>(i)] = static_cast<float>(rpm);
          motor_status_flags_[static_cast<size_t>(i)] = flags;
        }
      } else {
        const size_t idx = static_cast<size_t>(motor_index);
        motor_state_rpm_[idx] = static_cast<float>(rpm);
        motor_status_flags_[idx] = flags;
      }
    } else if (frame.dlc >= 7) {
      const int32_t rpm = decodeI32(frame.data[2], frame.data[3], frame.data[4], frame.data[5]);

      if (motor_index == broadcast_index_) {
        for (int i = 0; i < motor_count_; ++i) {
          motor_state_rpm_[static_cast<size_t>(i)] = static_cast<float>(rpm);
        }
      } else {
        motor_state_rpm_[static_cast<size_t>(motor_index)] = static_cast<float>(rpm);
      }
    }
  }

  publishTelemetry();
}

void CanInterfaceNode::publishTelemetry() {
  std::vector<float> rpm_snapshot;
  std::vector<uint8_t> flags_snapshot;
  {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    rpm_snapshot = motor_state_rpm_;
    flags_snapshot = motor_status_flags_;
  }

  std_msgs::Float32MultiArray rpm;
  rpm.data = rpm_snapshot;
  motor_state_pub_.publish(rpm);

  std_msgs::UInt8MultiArray mflag;
  mflag.data = flags_snapshot;
  motor_status_flag_pub_.publish(mflag);
}

void CanInterfaceNode::canReceiveThread() {
  while (running_.load() && ros::ok()) {
    if (socket_fd_ < 0) {
      ros::Duration(0.2).sleep();
      continue;
    }

    struct can_frame raw_frame;
    const int nbytes = read(socket_fd_, &raw_frame, sizeof(raw_frame));

    if (nbytes < 0) {
      if (errno == EINTR) {
        continue;
      }
      ROS_ERROR_THROTTLE(1.0, "CAN read error: %s", std::strerror(errno));
      closeSocket();
      ros::Duration(0.2).sleep();
      continue;
    }

    if (nbytes != static_cast<int>(sizeof(raw_frame))) {
      continue;
    }

    last_rx_time_ = ros::Time::now();

    can_msgs::Frame frame_msg;
    frame_msg.header.stamp = last_rx_time_;
    frame_msg.id = (raw_frame.can_id & CAN_EFF_FLAG) ? (raw_frame.can_id & CAN_EFF_MASK)
                             : (raw_frame.can_id & CAN_SFF_MASK);
    frame_msg.is_rtr = (raw_frame.can_id & CAN_RTR_FLAG) != 0;
    frame_msg.is_extended = (raw_frame.can_id & CAN_EFF_FLAG) != 0;
    frame_msg.is_error = (raw_frame.can_id & CAN_ERR_FLAG) != 0;
    frame_msg.dlc = raw_frame.can_dlc;
    std::copy(raw_frame.data, raw_frame.data + 8, frame_msg.data.begin());
    can_rx_pub_.publish(frame_msg);

    parseStatusFrame(frame_msg);
  }
}

void CanInterfaceNode::timerCallback(const ros::TimerEvent&) {
  if (socket_fd_ < 0) {
    openSocket();
    return;
  }

  if ((ros::Time::now() - last_rx_time_).toSec() > heartbeat_timeout_sec_) {
    ROS_WARN_THROTTLE(1.0, "CAN heartbeat timeout: no RX frame for %.2f s", heartbeat_timeout_sec_);
      std_msgs::Bool estop;
      estop.data = true;
      emergency_stop_pub_.publish(estop);
      return;
  }

    std_msgs::Bool estop;
    estop.data = false;
    emergency_stop_pub_.publish(estop);
}

}  // namespace can_motor_interface

int main(int argc, char** argv) {
  ros::init(argc, argv, "can_interface_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  can_motor_interface::CanInterfaceNode node(nh, pnh);
  ros::spin();
  return 0;
}
