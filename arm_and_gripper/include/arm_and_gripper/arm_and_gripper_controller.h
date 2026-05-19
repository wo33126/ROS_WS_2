#pragma once

#include <atomic>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <arm_and_gripper/PlaceMedicine.h>
#include <arm_and_gripper/ServoCommand.h>
#include <arm_and_gripper/ArmPlaceMedicine.h>

namespace arm_and_gripper {

/**
 * @brief 药品摆放动作控制器
 *
 * 协调机械臂电机（Y42/CAN2）和两个舵机，执行药品放置动作序列：
 *   1. 机械臂旋转到指定角度
 *   2. 舵机1张开 → 延时0.5s
 *   3. 舵机2张开 → 延时1.0s
 *   4. 可选：机械臂复位
 *
 * 触发方式：
 *   - 自动：订阅 /fine_tuning_done（微调完成后自动执行）
 *   - 手动：调用 /place_medicine 服务
 */
class ArmAndGripperController {
 public:
  ArmAndGripperController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ArmAndGripperController();

  bool init();

 private:
  // ── 回调 ──
  void fineTuningDoneCallback(const std_msgs::Bool::ConstPtr& msg);
  bool placeMedicineCallback(
      arm_and_gripper::PlaceMedicine::Request& req,
      arm_and_gripper::PlaceMedicine::Response& res);
  bool armPlaceMedicineCallback(
      arm_and_gripper::ArmPlaceMedicine::Request& req,
      arm_and_gripper::ArmPlaceMedicine::Response& res);

  // ── 动作 ──
  bool executePlaceSequence(double arm_angle,
                            double servo1_angle,
                            double servo2_angle,
                            bool reset_arm);

  // ── 硬件控制 ──
  bool initCanSocket();
  void closeCanSocket();
  bool sendArmAngleCommand(double angle_deg);
  bool sendArmLinearCommand(double distance_m);   // 新增: 线性推出/退回
  bool callServoAngle(uint8_t servo_id, double angle_deg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber fine_tuning_done_sub_;
  ros::ServiceServer place_medicine_srv_;
  ros::ServiceServer arm_place_medicine_srv_;   // 新增
  ros::ServiceClient servo_cmd_client_;

  // ── CAN 参数 (CAN2, 地址5) ──
  std::string can_device_;
  uint32_t arm_can_id_;        // 机械臂电机 CAN ID
  uint8_t position_cmd_code_;  // 位置模式指令码
  double arm_angle_scale_;     // 角度→编码器计数值比例
  int socket_fd_;

  // ── 动作参数（可从 yaml 配置覆盖） ──
  double default_arm_angle_;
  double default_servo1_open_angle_;
  double default_servo2_open_angle_;
  bool   default_reset_arm_;
  double post_servo1_delay_s_;
  double post_servo2_delay_s_;
  double arm_move_timeout_s_;

  // ── 推出动作参数 ──
  double push_distance_m_;        // 推出距离 (m)，默认 0.05
  double push_velocity_;          // 推出线速度 (m/s)，默认 0.03
  double push_timeout_s_;         // 推出超时 (s)，默认 5.0
  int8_t push_cmd_code_;          // 线性移动指令码

  // ── 状态 ──
  std::atomic<bool> sequence_running_;
  std::mutex seq_mutex_;
};

}  // namespace arm_and_gripper
