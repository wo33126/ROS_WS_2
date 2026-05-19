#include "path_manager/path_tracker.h"

#include <cmath>
#include <sstream>

namespace path_manager {

namespace {
constexpr double kPi = M_PI;
constexpr double kTwoPi = 2.0 * M_PI;
}  // namespace

// ── 构造 ───────────────────────────────────────────────────
PathTracker::PathTracker(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , current_point_idx_(0)
    , all_points_received_(false)
    , path_completed_(false)
    , current_x_(0.0)
    , current_y_(0.0)
    , current_yaw_(0.0)
    , has_odom_(false)
    , control_rate_hz_(30.0)
    , kp_linear_(0.5)
    , kp_angular_(1.5)
    , max_linear_vel_(1.0)
    , max_angular_vel_(2.0)
    , position_tolerance_(0.05)
    , heading_tolerance_(0.08)
    , use_turn_then_move_(false)
    , arrival_hold_time_(0.5)
    , state_(State::WAITING_FOR_PATH)
{}

// ── 初始化 ─────────────────────────────────────────────────
bool PathTracker::init() {
  // ── 参数读取 ──
  pnh_.param<double>("control_rate_hz", control_rate_hz_, 30.0);
  pnh_.param<double>("kp_linear", kp_linear_, 0.5);
  pnh_.param<double>("kp_angular", kp_angular_, 1.5);
  pnh_.param<double>("max_linear_vel", max_linear_vel_, 1.0);
  pnh_.param<double>("max_angular_vel", max_angular_vel_, 2.0);
  pnh_.param<double>("position_tolerance", position_tolerance_, 0.05);
  pnh_.param<double>("heading_tolerance", heading_tolerance_, 0.08);
  pnh_.param<bool>("use_turn_then_move", use_turn_then_move_, false);
  pnh_.param<double>("arrival_hold_time", arrival_hold_time_, 0.5);

  // 也支持从全局参数读取速度限制（与 robot_params.yaml 一致）
  nh_.param("/robot/max_linear_vel", max_linear_vel_, max_linear_vel_);
  nh_.param("/robot/max_angular_vel", max_angular_vel_, max_angular_vel_);

  // ── 订阅与发布 ──
  path_points_sub_ = nh_.subscribe("/path_points", 10,
                                   &PathTracker::pathPointsCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 10,
                            &PathTracker::odomCallback, this);

  // 发布到 cmd_vel_mux 的 external 输入（优先级: safety > external > teleop > fixed_route）
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_external", 10);
  path_finished_pub_ = nh_.advertise<std_msgs::Bool>("/path_finished", 10, true);  // latched

  // ── 控制定时器 ──
  const double period = 1.0 / std::max(1.0, control_rate_hz_);
  control_timer_ = nh_.createTimer(ros::Duration(period),
                                   &PathTracker::controlTimerCallback, this);

  ROS_INFO("[path_tracker] 初始化完成");
  ROS_INFO("[path_tracker]   控制频率: %.1f Hz", control_rate_hz_);
  ROS_INFO("[path_tracker]   控制策略: %s", use_turn_then_move_ ? "先转向后前进" : "同时控制");
  ROS_INFO("[path_tracker]   Kp_linear=%.2f, Kp_angular=%.2f", kp_linear_, kp_angular_);
  ROS_INFO("[path_tracker]   位置容差=%.3f m, 朝向容差=%.3f rad", position_tolerance_, heading_tolerance_);
  ROS_INFO("[path_tracker]   最大速度: v=%.2f m/s, ω=%.2f rad/s", max_linear_vel_, max_angular_vel_);
  ROS_INFO("[path_tracker]   等待 /path_points ...");

  // 发布初始 path_finished = false
  std_msgs::Bool init_msg;
  init_msg.data = false;
  path_finished_pub_.publish(init_msg);

  return true;
}

// ── /path_points 回调 ──────────────────────────────────────
void PathTracker::pathPointsCallback(const path_manager::PathPoint::ConstPtr& msg) {
  // 过滤重复点：如果与队列最后一个点完全相同则忽略
  if (!point_queue_.empty()) {
    const auto& last = point_queue_.back();
    if (std::abs(last.x - msg->x) < 1e-6 &&
        std::abs(last.y - msg->y) < 1e-6) {
      return;  // 重复点，忽略
    }
  }

  // 如果已经完成，新路径点会触发新路径
  if (state_ == State::ALL_DONE) {
    ROS_INFO("[path_tracker] 收到新路径点，重置跟踪状态");
    point_queue_.clear();
    current_point_idx_ = 0;
    path_completed_ = false;
    all_points_received_ = false;

    // 清除 path_finished
    std_msgs::Bool msg;
    msg.data = false;
    path_finished_pub_.publish(msg);
  }

  point_queue_.push_back(*msg);

  if (state_ == State::WAITING_FOR_PATH || state_ == State::STOPPED) {
    state_ = State::MOVING;
    ROS_INFO("[path_tracker] 开始跟踪路径，共 %zu 个点", point_queue_.size());
  }
}

// ── /odom 回调 ─────────────────────────────────────────────
void PathTracker::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  // 从四元数提取 yaw
  const auto& q = msg->pose.pose.orientation;
  double siny = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  current_yaw_ = std::atan2(siny, cosy);

  has_odom_ = true;
}

// ── 控制定时器 ─────────────────────────────────────────────
void PathTracker::controlTimerCallback(const ros::TimerEvent& /*event*/) {
  if (!has_odom_) {
    ROS_WARN_THROTTLE(2.0, "[path_tracker] 等待 /odom 数据...");
    return;
  }

  // 已完成则停止输出
  if (state_ == State::ALL_DONE) {
    return;
  }

  // 没有路径点
  if (point_queue_.empty()) {
    if (state_ != State::WAITING_FOR_PATH) {
      state_ = State::WAITING_FOR_PATH;
    }
    return;
  }

  // 所有点已完成
  if (current_point_idx_ >= point_queue_.size()) {
    if (!path_completed_) {
      path_completed_ = true;
      state_ = State::ALL_DONE;
      publishStop();

      std_msgs::Bool done_msg;
      done_msg.data = true;
      path_finished_pub_.publish(done_msg);

      ROS_INFO("[path_tracker] ====== 全部路径点完成！======");
    }
    return;
  }

  const path_manager::PathPoint& target = point_queue_[current_point_idx_];

  // 使用路径点自带的 tolerance，若未设置则用默认值
  double tolerance = (target.tolerance > 0.0) ? target.tolerance : position_tolerance_;

  // 检查是否到达
  bool reached = isPointReached(target, current_x_, current_y_);

  // ── 状态机 ──
  switch (state_) {
    case State::TURNING: {
      // 计算朝向误差
      double dx = target.x - current_x_;
      double dy = target.y - current_y_;
      double target_heading = std::atan2(dy, dx);
      double heading_err = normalizeAngle(target_heading - current_yaw_);

      if (std::abs(heading_err) < heading_tolerance_) {
        // 朝向已对齐，切换到移动
        state_ = State::MOVING;
        ROS_DEBUG("[path_tracker] 朝向已对齐，开始前进到点 %lu", current_point_idx_ + 1);
      } else {
        // 仅旋转
        double cmd_omega = kp_angular_ * heading_err;
        cmd_omega = clamp(cmd_omega, -max_angular_vel_, max_angular_vel_);

        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.angular.z = cmd_omega;
        cmd_vel_pub_.publish(twist);
      }
      break;
    }

    case State::MOVING: {
      if (reached) {
        // 初次到达，开始计时
        state_ = State::ARRIVED_AT_POINT;
        arrival_time_ = ros::Time::now();
        ROS_INFO("[path_tracker] 到达点 %lu/%zu (x=%.3f, y=%.3f)，稳定中...",
                 current_point_idx_ + 1, point_queue_.size(),
                 target.x, target.y);
        // 发布零速度让机器人停下
        publishStop();
        break;
      }

      // 计算并发布控制量
      double cmd_vx = 0.0, cmd_omega = 0.0;
      computeControl(target, current_x_, current_y_, current_yaw_,
                     cmd_vx, cmd_omega);

      geometry_msgs::Twist twist;
      twist.linear.x = cmd_vx;
      twist.linear.y = 0.0;
      twist.angular.z = cmd_omega;
      cmd_vel_pub_.publish(twist);
      break;
    }

    case State::ARRIVED_AT_POINT: {
      // 保持零速度，等待稳定
      publishStop();

      ros::Duration hold_dur = ros::Time::now() - arrival_time_;
      if (hold_dur.toSec() >= arrival_hold_time_) {
        // 切换到下一个点
        current_point_idx_++;

        if (current_point_idx_ >= point_queue_.size()) {
          // 全部完成
          path_completed_ = true;
          state_ = State::ALL_DONE;

          std_msgs::Bool done_msg;
          done_msg.data = true;
          path_finished_pub_.publish(done_msg);

          ROS_INFO("[path_tracker] ====== 全部路径点完成！======");
        } else {
          // 下一个点
          const auto& next = point_queue_[current_point_idx_];
          ROS_INFO("[path_tracker] → 切换到点 %lu/%zu (x=%.3f, y=%.3f)",
                   current_point_idx_ + 1, point_queue_.size(),
                   next.x, next.y);

          if (use_turn_then_move_) {
            state_ = State::TURNING;
          } else {
            state_ = State::MOVING;
          }
        }
      }
      break;
    }

    default:
      break;
  }
}

// ── 计算控制量 ─────────────────────────────────────────────
void PathTracker::computeControl(const path_manager::PathPoint& target,
                                 double current_x, double current_y,
                                 double current_yaw,
                                 double& cmd_vx, double& cmd_omega) {
  double dx = target.x - current_x;
  double dy = target.y - current_y;
  double distance = std::sqrt(dx * dx + dy * dy);

  double target_heading = std::atan2(dy, dx);
  double heading_err = normalizeAngle(target_heading - current_yaw);

  if (use_turn_then_move_) {
    // 先转向后前进：如果朝向误差大则仅旋转
    if (std::abs(heading_err) > heading_tolerance_) {
      cmd_vx = 0.0;
      cmd_omega = kp_angular_ * heading_err;
    } else {
      // 前进 + 小幅修正朝向
      cmd_vx = kp_linear_ * distance;
      cmd_omega = kp_angular_ * heading_err;
    }
  } else {
    // 同时控制：线速度随朝向偏差衰减（越对准越快）
    double heading_factor = std::cos(heading_err);
    // 确保不会因为没对准而完全不动（至少保留一点前进分量）
    heading_factor = std::max(0.2, heading_factor);

    cmd_vx = kp_linear_ * distance * heading_factor;
    cmd_omega = kp_angular_ * heading_err;
  }

  // 限幅
  cmd_vx = clamp(cmd_vx, -max_linear_vel_, max_linear_vel_);
  cmd_omega = clamp(cmd_omega, -max_angular_vel_, max_angular_vel_);

  // 接近目标时减速（soft slowdown）
  double slowdown_dist = position_tolerance_ * 3.0;
  if (distance < slowdown_dist) {
    double scale = distance / slowdown_dist;
    // 保持最低速度避免卡住
    scale = std::max(0.05, scale);
    cmd_vx *= scale;
  }
}

// ── 到达判定 ───────────────────────────────────────────────
bool PathTracker::isPointReached(const path_manager::PathPoint& target,
                                 double current_x, double current_y) const {
  double dx = target.x - current_x;
  double dy = target.y - current_y;
  double dist = std::sqrt(dx * dx + dy * dy);

  double tolerance = (target.tolerance > 0.0) ? target.tolerance : position_tolerance_;
  return dist < tolerance;
}

// ── 角度归一化 ─────────────────────────────────────────────
double PathTracker::normalizeAngle(double angle) const {
  while (angle > kPi)  angle -= kTwoPi;
  while (angle < -kPi) angle += kTwoPi;
  return angle;
}

// ── 限幅 ───────────────────────────────────────────────────
double PathTracker::clamp(double val, double min_val, double max_val) const {
  if (val > max_val) return max_val;
  if (val < min_val) return min_val;
  return val;
}

// ── 安全停止 ───────────────────────────────────────────────
void PathTracker::publishStop() {
  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.angular.z = 0.0;
  cmd_vel_pub_.publish(twist);
}

}  // namespace path_manager

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "path_tracker_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  path_manager::PathTracker tracker(nh, pnh);
  if (!tracker.init()) {
    ROS_FATAL("[path_tracker] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
