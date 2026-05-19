#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>

namespace fine_tuning {

/**
 * @brief 终点位置微调控制器
 *
 * 订阅 /path_finished（path_tracker_node 输出）和 /vl53l1x_distance（STM32 上报距离，mm）。
 * 当路径跟踪完成后，根据 VL53L1X 传感器距离值进行位置微调，使距离逼近目标值。
 *
 * 工作原理：
 *   1. 收到 /path_finished == true 后进入 FINE_TUNING 状态
 *   2. 读取当前距离，与 target_distance_mm 比较
 *   3. 若误差超过 tolerance_mm，沿指定轴发布微小的 cmd_vel 指令
 *   4. 采用「步进+再测量」策略：每发布一小段移动指令后停顿测量，避免超调
 *   5. 误差进入容差范围后发布 /fine_tuning_done，停止控制
 *
 * 速度发布到 /cmd_vel_external，与现有 cmd_vel_mux 优先级体系兼容
 * （safety > external > teleop > fixed_route）。
 */
class FineTuningController {
 public:
  FineTuningController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~FineTuningController() = default;

  bool init();

 private:
  // ── 回调 ──
  void pathFinishedCallback(const std_msgs::Bool::ConstPtr& msg);
  void distanceCallback(const std_msgs::Float32::ConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);
  bool fineTuningStartCallback(std_srvs::Trigger::Request& req,
                               std_srvs::Trigger::Response& res);

  // ── 辅助 ──
  void publishStop();
  void publishVelocity(double vx, double vy);

  // ── 工具函数 ──
  double clamp(double val, double lo, double hi) const {
    return std::max(lo, std::min(hi, val));
  }

  // ── ROS 接口 ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber path_finished_sub_;
  ros::Subscriber distance_sub_;
  ros::Publisher  cmd_vel_pub_;
  ros::Publisher  fine_tuning_done_pub_;
  ros::Timer      control_timer_;

  // ── 服务 ──
  ros::ServiceServer fine_tuning_start_srv_;

  // ── 状态机 ──
  enum class State {
    IDLE,            // 等待路径完成信号
    FINE_TUNING,     // 正在微调
    DONE             // 微调完成
  };
  State state_;

  // ── 传感器数据 ──
  double current_distance_mm_;
  bool   has_distance_;

  // ── 参数 ──
  double target_distance_mm_;      // 目标距离 (mm)，默认 200
  double tolerance_mm_;            // 允许误差 (mm)，默认 ±10
  double move_axis_;               // 移动轴：1=X轴, 2=Y轴
  int    move_direction_;          // 移动方向：+1 或 -1（传感器装在哪个方向）
  double step_velocity_;           // 步进移动速度 (m/s)，默认 0.05
  double step_duration_;           // 每次步进持续时间 (s)，默认 0.2
  double settle_time_;             // 步进后测量稳定等待时间 (s)，默认 0.3
  double control_rate_hz_;         // 控制循环频率 (Hz)，默认 10
  double max_fine_tuning_time_;    // 微调最大时长 (s)，超时则强制结束，默认 30
  double kp_distance_;             // 距离误差比例增益（将 mm 误差映射到速度）
  int    max_steps_;               // 最大微调步数，超过则失败 (0=无限制)
  int    current_step_;            // 当前步数计数

  // ── 步进状态 ──
  ros::Time step_start_time_;      // 当前步进开始时刻
  ros::Time settle_start_time_;    // 当前测量稳定开始时刻
  bool      is_moving_;            // 是否正在执行步进移动
  bool      is_settling_;          // 是否正在等待测量稳定
  bool      path_finished_received_; // 是否已收到终点完成信号
  ros::Time tuning_start_time_;    // 微调开始时刻
};

}  // namespace fine_tuning
