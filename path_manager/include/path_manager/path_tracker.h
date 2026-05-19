#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <path_manager/PathPoint.h>

namespace path_manager {

/**
 * @brief 路径跟踪控制器
 *
 * 订阅 /path_points（来自 path_manager_node）和 /odom（来自 base_odometry_node），
 * 使用比例控制器依次跟踪每个路径点，到达后自动切换到下一个点。
 * 控制指令发布到 /cmd_vel_external，与 cmd_vel_mux 兼容。
 * 全部路径点完成后发布 /path_finished = true。
 */
class PathTracker {
 public:
  PathTracker(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~PathTracker() = default;

  bool init();

 private:
  // ── 回调 ──
  void pathPointsCallback(const path_manager::PathPoint::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);

  // ── 控制逻辑 ──
  void computeControl(const path_manager::PathPoint& target,
                      double current_x, double current_y, double current_yaw,
                      double& cmd_vx, double& cmd_omega);
  bool isPointReached(const path_manager::PathPoint& target,
                      double current_x, double current_y) const;
  double normalizeAngle(double angle) const;
  double clamp(double val, double min_val, double max_val) const;

  // ── 安全停止 ──
  void publishStop();

  // ── ROS 接口 ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber path_points_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher  cmd_vel_pub_;
  ros::Publisher  path_finished_pub_;
  ros::Timer      control_timer_;

  // ── 路径点队列 ──
  std::vector<path_manager::PathPoint> point_queue_;
  size_t current_point_idx_;
  bool all_points_received_;  // 是否已收到全部路径点（话题可能持续发布）
  bool path_completed_;

  // ── 当前位姿 ──
  double current_x_;
  double current_y_;
  double current_yaw_;
  bool   has_odom_;

  // ── 控制参数 ──
  double control_rate_hz_;       // 控制循环频率 (Hz)
  double kp_linear_;             // 线速度比例增益
  double kp_angular_;            // 角速度比例增益
  double max_linear_vel_;        // 最大线速度 (m/s)
  double max_angular_vel_;       // 最大角速度 (rad/s)
  double position_tolerance_;    // 默认到达判定距离 (m)
  double heading_tolerance_;     // 朝向对齐容差 (rad)，用于 turn-then-move 模式
  bool   use_turn_then_move_;    // true=先转向后前进, false=同时控制
  double arrival_hold_time_;     // 到达后稳定保持时间 (s)
  ros::Time arrival_time_;       // 进入到达状态的时刻

  // ── 状态机 ──
  enum class State {
    WAITING_FOR_PATH,   // 等待路径点
    TURNING,            // 正在转向（仅 turn-then-move 模式）
    MOVING,             // 正在移动
    ARRIVED_AT_POINT,   // 已到达当前点，等待切换
    ALL_DONE,           // 全部路径点完成
    STOPPED             // 安全停止
  };
  State state_;
};

}  // namespace path_manager
