#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace arm_interface {

class ArmControllerNode {
 public:
  ArmControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ArmControllerNode();

 private:
  void armCmdCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void canReceiveThread();

  bool openSocket();
  void closeSocket();
  bool sendJointCommand(uint8_t joint_idx, double position_rad);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber joint_cmd_sub_;
  ros::Publisher arm_state_pub_;

  std::string can_device_;
  int socket_fd_;
  std::atomic<bool> running_;
  std::thread rx_thread_;

  uint32_t tx_can_base_id_;
  uint32_t rx_can_base_id_;
  uint8_t set_joint_cmd_code_;
  double position_scale_counts_;

  int joint_count_;
  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;
  std::vector<double> current_positions_;
  std::mutex state_mutex_;
};

}  // namespace arm_interface
