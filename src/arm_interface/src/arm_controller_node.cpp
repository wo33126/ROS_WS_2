#include "arm_interface/arm_controller.h"

#include <cerrno>
#include <cmath>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace arm_interface {

ArmControllerNode::ArmControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      socket_fd_(-1),
      running_(true),
      tx_can_base_id_(0x200),
      rx_can_base_id_(0x300),
      set_joint_cmd_code_(0x10),
      position_scale_counts_(1000.0),
      joint_count_(6) {
  pnh_.param<std::string>("can_device", can_device_, "can0");
  int tx_can_base_id_i = static_cast<int>(tx_can_base_id_);
  int rx_can_base_id_i = static_cast<int>(rx_can_base_id_);
  int set_joint_cmd_code_i = static_cast<int>(set_joint_cmd_code_);
  pnh_.param("tx_can_base_id", tx_can_base_id_i, tx_can_base_id_i);
  pnh_.param("rx_can_base_id", rx_can_base_id_i, rx_can_base_id_i);
  pnh_.param("set_joint_cmd_code", set_joint_cmd_code_i, set_joint_cmd_code_i);
  tx_can_base_id_ = static_cast<uint32_t>(tx_can_base_id_i);
  rx_can_base_id_ = static_cast<uint32_t>(rx_can_base_id_i);
  set_joint_cmd_code_ = static_cast<uint8_t>(set_joint_cmd_code_i);
  pnh_.param("position_scale_counts", position_scale_counts_, position_scale_counts_);

  nh_.param("/arm/joint_count", joint_count_, joint_count_);
  pnh_.param("joint_count", joint_count_, joint_count_);

  if (!nh_.getParam("/arm/joint_names", joint_names_)) {
    pnh_.getParam("joint_names", joint_names_);
  }
  if (joint_names_.empty()) {
    joint_names_.resize(joint_count_);
    for (int i = 0; i < joint_count_; ++i) {
      joint_names_[i] = "joint_" + std::to_string(i + 1);
    }
  }
  joint_count_ = static_cast<int>(joint_names_.size());
  joint_name_to_index_.clear();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_name_to_index_[joint_names_[i]] = i;
  }
  current_positions_.assign(joint_count_, 0.0);

  joint_cmd_sub_ = nh_.subscribe("/arm_joint_cmd", 20, &ArmControllerNode::armCmdCallback, this);
  arm_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/arm_state", 20);

  if (!openSocket()) {
    ROS_WARN("Arm CAN socket open failed at startup, receive thread will retry.");
  }

  rx_thread_ = std::thread(&ArmControllerNode::canReceiveThread, this);
}

ArmControllerNode::~ArmControllerNode() {
  running_.store(false);
  closeSocket();
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
}

bool ArmControllerNode::openSocket() {
  if (socket_fd_ >= 0) {
    return true;
  }

  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    ROS_ERROR("Arm socket create failed: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_device_.c_str());
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ROS_ERROR("Arm ioctl(SIOCGIFINDEX) failed for %s: %s", can_device_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR("Arm bind failed on %s: %s", can_device_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  socket_fd_ = fd;
  ROS_INFO("Arm SocketCAN connected on %s", can_device_.c_str());
  return true;
}

void ArmControllerNode::closeSocket() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool ArmControllerNode::sendJointCommand(uint8_t joint_idx, double position_rad) {
  if (socket_fd_ < 0 && !openSocket()) {
    return false;
  }

  const int32_t counts = static_cast<int32_t>(std::llround(position_rad * position_scale_counts_));

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = tx_can_base_id_ + joint_idx;
  frame.can_dlc = 8;
  frame.data[0] = set_joint_cmd_code_;
  frame.data[1] = joint_idx;
  frame.data[2] = static_cast<uint8_t>(counts & 0xFF);
  frame.data[3] = static_cast<uint8_t>((counts >> 8) & 0xFF);
  frame.data[4] = static_cast<uint8_t>((counts >> 16) & 0xFF);
  frame.data[5] = static_cast<uint8_t>((counts >> 24) & 0xFF);

  const int nbytes = write(socket_fd_, &frame, sizeof(frame));
  if (nbytes != static_cast<int>(sizeof(frame))) {
    ROS_ERROR_THROTTLE(1.0, "Arm CAN write failed: %s", std::strerror(errno));
    closeSocket();
    return false;
  }
  return true;
}

void ArmControllerNode::armCmdCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  if (msg->position.empty()) {
    return;
  }

  // 优先按关节名映射，避免上游 JointState 顺序变化导致控制错轴。
  if (!msg->name.empty() && msg->name.size() == msg->position.size()) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const auto it = joint_name_to_index_.find(msg->name[i]);
      if (it == joint_name_to_index_.end()) {
        continue;
      }
      const uint8_t joint_idx = static_cast<uint8_t>(it->second);
      if (!sendJointCommand(joint_idx, msg->position[i])) {
        ROS_WARN_THROTTLE(1.0, "Failed to send arm command for joint %s", msg->name[i].c_str());
      }
    }
    return;
  }

  const size_t count = std::min(static_cast<size_t>(joint_count_), msg->position.size());
  for (size_t i = 0; i < count; ++i) {
    if (!sendJointCommand(static_cast<uint8_t>(i), msg->position[i])) {
      ROS_WARN_THROTTLE(1.0, "Failed to send arm command for index %zu", i);
    }
  }
}

void ArmControllerNode::canReceiveThread() {
  while (running_.load() && ros::ok()) {
    if (socket_fd_ < 0) {
      openSocket();
      ros::Duration(0.2).sleep();
      continue;
    }

    struct can_frame raw;
    const int nbytes = read(socket_fd_, &raw, sizeof(raw));
    if (nbytes < 0) {
      if (errno == EINTR) {
        continue;
      }
      ROS_ERROR_THROTTLE(1.0, "Arm CAN read failed: %s", std::strerror(errno));
      closeSocket();
      ros::Duration(0.2).sleep();
      continue;
    }
    if (nbytes != static_cast<int>(sizeof(raw))) {
      continue;
    }

    const uint32_t id = raw.can_id & CAN_EFF_MASK;
    if (id < rx_can_base_id_ || id >= rx_can_base_id_ + static_cast<uint32_t>(joint_count_)) {
      continue;
    }
    if (raw.can_dlc < 4) {
      continue;
    }

    const uint8_t joint_idx = static_cast<uint8_t>(id - rx_can_base_id_);
    const int32_t counts =
        static_cast<int32_t>(static_cast<uint32_t>(raw.data[0]) |
                             (static_cast<uint32_t>(raw.data[1]) << 8) |
                             (static_cast<uint32_t>(raw.data[2]) << 16) |
                             (static_cast<uint32_t>(raw.data[3]) << 24));
    const double pos_rad = static_cast<double>(counts) / position_scale_counts_;

    sensor_msgs::JointState state;
    state.header.stamp = ros::Time::now();
    state.name = joint_names_;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (joint_idx < current_positions_.size()) {
        current_positions_[joint_idx] = pos_rad;
      }
      state.position = current_positions_;
    }

    arm_state_pub_.publish(state);
  }
}

}  // namespace arm_interface

int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  arm_interface::ArmControllerNode node(nh, pnh);
  ros::spin();
  return 0;
}
