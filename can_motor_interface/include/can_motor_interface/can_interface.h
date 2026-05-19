#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Bool.h>

namespace can_motor_interface {

class CanInterfaceNode {
 public:
  CanInterfaceNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~CanInterfaceNode();

 private:
  void cmdCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);
  void canReceiveThread();

  bool openSocket();
  void closeSocket();
  bool sendSpeedCommand(uint8_t motor_index, float target_rpm);
  bool sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t dlc);
  uint8_t calcChecksum(const uint8_t* data, uint8_t len) const;
  void parseStatusFrame(const can_msgs::Frame& frame);
  void publishTelemetry();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cmd_sub_;
  ros::Publisher motor_state_pub_;
  ros::Publisher motor_status_flag_pub_;
  ros::Publisher emergency_stop_pub_;
  ros::Publisher can_rx_pub_;
  ros::Timer monitor_timer_;

  std::string can_device_;
  int socket_fd_;
  std::atomic<bool> running_;
  std::thread rx_thread_;

  std::vector<int> motor_ids_;
  std::vector<float> motor_state_rpm_;
  std::vector<uint8_t> motor_status_flags_;
  std::mutex telemetry_mutex_;

  uint32_t tx_can_id_;
  uint32_t rx_can_id_;
  bool rx_can_id_filter_enable_;
  bool use_extended_frame_;
  bool payload_little_endian_;
  bool checksum_use_sum8_;

  uint8_t speed_cmd_code_;
  uint8_t status_cmd_code_;
  uint8_t broadcast_index_;
  float max_rpm_;
  int motor_count_;

  double heartbeat_timeout_sec_;
  ros::Time last_rx_time_;
};

}  // namespace can_motor_interface
