#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

namespace robot_navigation {

/**
 * @brief 距离安全检测节点 — 增强版（支持侧向传感器 + Bug2 避障）
 *
 * 订阅 VL53L1X 距离数据（前方/左/右）和当前底盘速度，在障碍物过近时通过
 * /cmd_vel_safety 通道（cmd_vel_mux 最高优先级）下发：
 *   1. 减速/停止指令（基础安全）
 *   2. Bug2 绕障指令（动态避障）
 *
 * Bug2 算法：
 *   - 检测前方障碍 → 停止 → 检查左右空间 → 转向开阔侧
 *   - 沿障碍物边缘前进 → 到达目标方向 → 回归原始路径
 */
class DistanceSafetyNode {
 public:
  DistanceSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~DistanceSafetyNode() = default;

  bool init();

 private:
  // ── 回调 ──
  void distanceCallback(const std_msgs::Float32::ConstPtr& msg);
  void distanceLeftCallback(const std_msgs::Float32::ConstPtr& msg);
  void distanceRightCallback(const std_msgs::Float32::ConstPtr& msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void safetyTimerCallback(const ros::TimerEvent& event);

  // ── 工具 ──
  double clamp(double val, double lo, double hi) const;
  void publishSafetyStop();
  void publishSafetyReverse(double reverse_speed);
  void publishSafetyClear();
  void publishBug2Command(double vx, double vy, double vw);

  // ── Bug2 避障状态机 ──
  void bug2StateMachine();

  // ── ROS 接口 ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber distance_sub_;
  ros::Subscriber distance_left_sub_;
  ros::Subscriber distance_right_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher  cmd_vel_safety_pub_;
  ros::Publisher  safety_state_pub_;
  ros::Publisher  bug2_state_pub_;
  ros::Timer      safety_timer_;

  // ── 参数 ──
  double safety_distance_mm_;       // 安全停止距离 (mm)
  double critical_distance_mm_;     // 紧急后退距离 (mm)
  double reverse_speed_;            // 后退速度 (m/s)
  double distance_timeout_sec_;     // 传感器数据超时
  double control_rate_hz_;          // 控制频率
  bool   check_forward_only_;       // 仅检测前方

  // ── Bug2 参数 ──
  bool   bug2_enabled_;             // 启用 Bug2 避障
  double bug2_side_clearance_mm_;   // 侧向安全间隙 (mm)
  double bug2_turn_speed_;          // 避障转弯速度 (rad/s)
  double bug2_along_speed_;         // 沿障碍物行进速度 (m/s)
  double bug2_max_duration_sec_;    // Bug2 最大持续时长
  double bug2_goal_tolerance_rad_;  // 目标方向容差

  // ── 状态 ──
  double current_distance_mm_;
  double current_distance_left_mm_;
  double current_distance_right_mm_;
  double current_vx_;
  double current_vy_;
  bool   has_distance_;
  bool   has_distance_left_;
  bool   has_distance_right_;
  bool   has_cmd_vel_;
  bool   has_odom_;
  ros::Time last_distance_time_;
  ros::Time last_distance_left_time_;
  ros::Time last_distance_right_time_;

  // ── 里程计数据 ──
  double current_yaw_;              // 当前偏航角 (rad)
  double target_yaw_;               // Bug2 目标方向

  // ── 安全状态 ──
  enum class SafetyState {
    CLEAR,      // 安全，距离足够
    WARNING,    // 距离接近安全阈值，减速
    STOP,       // 距离过近，停止
    REVERSE,    // 距离极近，后退
    TIMEOUT     // 传感器数据超时
  };
  SafetyState safety_state_;

  // ── Bug2 状态机 ──
  enum class Bug2State {
    IDLE,             // 空闲，无避障
    STOPPED,          // 检测到障碍，已停止
    CHECKING_SIDES,   // 检查左右空间
    TURNING,          // 转向开阔侧
    MOVING_ALONG,     // 沿障碍物边缘行进
    BACK_TO_PATH,     // 回归原始路径
    FAILED            // 避障失败
  };
  Bug2State bug2_state_;
  ros::Time bug2_start_time_;
  bool bug2_turn_direction_;  // true=右转, false=左转
};

}  // namespace robot_navigation
