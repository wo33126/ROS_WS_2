/**
 * @file distance_safety_node.cpp
 * @brief 距离安全检测节点 — 障碍物过近时自动制动
 *
 * 通过 cmd_vel_mux 的 safety 通道（最高优先级）介入底盘控制，
 * 在 VL53L1X 检测到障碍物距离过近时自动停止或后退。
 */

#include "robot_navigation/distance_safety_node.h"

#include <cmath>
#include <sstream>

namespace robot_navigation {

// ── 构造 ───────────────────────────────────────────────────
DistanceSafetyNode::DistanceSafetyNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , safety_distance_mm_(300.0)
    , critical_distance_mm_(100.0)
    , reverse_speed_(0.05)
    , distance_timeout_sec_(0.5)
    , control_rate_hz_(20.0)
    , check_forward_only_(true)
    , bug2_enabled_(true)
    , bug2_side_clearance_mm_(400.0)
    , bug2_turn_speed_(0.5)
    , bug2_along_speed_(0.1)
    , bug2_max_duration_sec_(10.0)
    , bug2_goal_tolerance_rad_(0.2)
    , current_distance_mm_(0.0)
    , current_distance_left_mm_(9999.0)
    , current_distance_right_mm_(9999.0)
    , current_vx_(0.0)
    , current_vy_(0.0)
    , has_distance_(false)
    , has_distance_left_(false)
    , has_distance_right_(false)
    , has_cmd_vel_(false)
    , has_odom_(false)
    , last_distance_time_(ros::Time::now())
    , current_yaw_(0.0)
    , target_yaw_(0.0)
    , safety_state_(SafetyState::CLEAR)
    , bug2_state_(Bug2State::IDLE)
    , bug2_turn_direction_(false)
{}

// ── 初始化 ─────────────────────────────────────────────────
bool DistanceSafetyNode::init() {
  // ── 参数读取 ──
  pnh_.param<double>("safety_distance_mm", safety_distance_mm_, 300.0);
  pnh_.param<double>("critical_distance_mm", critical_distance_mm_, 100.0);
  pnh_.param<double>("reverse_speed", reverse_speed_, 0.05);
  pnh_.param<double>("distance_timeout_sec", distance_timeout_sec_, 0.5);
  pnh_.param<double>("control_rate_hz", control_rate_hz_, 20.0);
  pnh_.param<bool>("check_forward_only", check_forward_only_, true);

  // ── Bug2 避障参数 ──
  pnh_.param<bool>("bug2_enabled", bug2_enabled_, true);
  pnh_.param<double>("bug2_side_clearance_mm", bug2_side_clearance_mm_, 400.0);
  pnh_.param<double>("bug2_turn_speed", bug2_turn_speed_, 0.5);
  pnh_.param<double>("bug2_along_speed", bug2_along_speed_, 0.1);
  pnh_.param<double>("bug2_max_duration_sec", bug2_max_duration_sec_, 10.0);
  pnh_.param<double>("bug2_goal_tolerance_rad", bug2_goal_tolerance_rad_, 0.2);

  // 确保 threshold 大小逻辑正确
  if (critical_distance_mm_ >= safety_distance_mm_) {
    ROS_WARN("[distance_safety] critical_distance_mm (%.0f) >= safety_distance_mm (%.0f), "
             "自动调整 critical = safety * 0.33",
             critical_distance_mm_, safety_distance_mm_);
    critical_distance_mm_ = safety_distance_mm_ * 0.33;
  }

  // ── 订阅 ──
  distance_sub_ = nh_.subscribe("/vl53l1x_distance", 10,
                                &DistanceSafetyNode::distanceCallback, this);

  // 侧向传感器（可选，地址不同）
  distance_left_sub_ = nh_.subscribe("/vl53l1x_distance_left", 10,
                                     &DistanceSafetyNode::distanceLeftCallback, this);
  distance_right_sub_ = nh_.subscribe("/vl53l1x_distance_right", 10,
                                      &DistanceSafetyNode::distanceRightCallback, this);

  // 订阅最终下发的 cmd_vel（经过 mux 后），用于判断运动方向
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10,
                               &DistanceSafetyNode::cmdVelCallback, this);

  // 订阅里程计用于 Bug2 目标方向计算
  odom_sub_ = nh_.subscribe("/odom", 10,
                            &DistanceSafetyNode::odomCallback, this);

  // ── 发布 ──
  // 发布到 cmd_vel_mux 的 safety 通道（最高优先级）
  cmd_vel_safety_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_safety", 10);
  safety_state_pub_   = nh_.advertise<std_msgs::String>("/distance_safety/state", 10, true);
  bug2_state_pub_     = nh_.advertise<std_msgs::String>("/distance_safety/bug2_state", 10, true);

  // ── 定时器 ──
  const double period = 1.0 / std::max(1.0, control_rate_hz_);
  safety_timer_ = nh_.createTimer(ros::Duration(period),
                                  &DistanceSafetyNode::safetyTimerCallback, this);

  // ── 初始发布安全状态 ──
  {
    std_msgs::String init_state;
    init_state.data = "CLEAR";
    safety_state_pub_.publish(init_state);
  }
  // 初始发布一帧空指令，让 mux 知道 safety 通道存在
  publishSafetyClear();

  ROS_INFO("[distance_safety] 初始化完成");
  ROS_INFO("[distance_safety]   安全停止: %.0f mm, 紧急后退: %.0f mm, 后退速度: %.2f m/s",
           safety_distance_mm_, critical_distance_mm_, reverse_speed_);
  ROS_INFO("[distance_safety]   传感器超时: %.1f s, 控制频率: %.1f Hz",
           distance_timeout_sec_, control_rate_hz_);
  ROS_INFO("[distance_safety]   模式: %s", check_forward_only_ ? "仅检测前方" : "全方向检测");
  if (bug2_enabled_) {
    ROS_INFO("[distance_safety]   Bug2 避障: 启用 (侧向间隙: %.0f mm, 最大持续: %.0f s)",
             bug2_side_clearance_mm_, bug2_max_duration_sec_);
  }

  return true;
}

// ── 距离回调 ───────────────────────────────────────────────
void DistanceSafetyNode::distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
  current_distance_mm_ = static_cast<double>(msg->data);
  has_distance_ = true;
  last_distance_time_ = ros::Time::now();
}

// ── 左侧距离回调 ──────────────────────────────────────────
void DistanceSafetyNode::distanceLeftCallback(const std_msgs::Float32::ConstPtr& msg) {
  current_distance_left_mm_ = static_cast<double>(msg->data);
  has_distance_left_ = true;
  last_distance_left_time_ = ros::Time::now();
}

// ── 右侧距离回调 ──────────────────────────────────────────
void DistanceSafetyNode::distanceRightCallback(const std_msgs::Float32::ConstPtr& msg) {
  current_distance_right_mm_ = static_cast<double>(msg->data);
  has_distance_right_ = true;
  last_distance_right_time_ = ros::Time::now();
}

// ── 里程计回调 ────────────────────────────────────────────
void DistanceSafetyNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  // 从四元数提取偏航角
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
                             1.0 - 2.0 * (qy * qy + qz * qz));
  has_odom_ = true;
}

// ── cmd_vel 回调（用于判断运动方向）────────────────────────
void DistanceSafetyNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  current_vx_ = msg->linear.x;
  current_vy_ = msg->linear.y;
  has_cmd_vel_ = true;
}

// ── 安全检测定时器 ─────────────────────────────────────────
void DistanceSafetyNode::safetyTimerCallback(const ros::TimerEvent& /*event*/) {
  const ros::Time now = ros::Time::now();

  // ── 检查传感器超时 ──
  double time_since_last = (now - last_distance_time_).toSec();
  if (!has_distance_ || time_since_last > distance_timeout_sec_) {
    if (safety_state_ != SafetyState::TIMEOUT) {
      safety_state_ = SafetyState::TIMEOUT;
      publishSafetyStop();
      std_msgs::String state;
      state.data = "TIMEOUT";
      safety_state_pub_.publish(state);
      ROS_WARN_THROTTLE(1.0, "[distance_safety] 传感器数据超时 (%.1f s)，触发安全停止",
                        time_since_last);
    }
    return;
  }

  // ── 方向判断 ──
  double effective_vx = current_vx_;
  if (!has_cmd_vel_) {
    // 尚无 cmd_vel 数据，保守假设静止
    effective_vx = 0.0;
  }

  // 仅检测前方时，如果机器人不向前则不干预
  if (check_forward_only_ && effective_vx <= 0.001) {
    if (safety_state_ != SafetyState::CLEAR) {
      safety_state_ = SafetyState::CLEAR;
      publishSafetyClear();
      std_msgs::String state;
      state.data = "CLEAR";
      safety_state_pub_.publish(state);
    }
    return;
  }

  // ── 距离判定 ──
  if (current_distance_mm_ <= critical_distance_mm_) {
    // 紧急：距离极近，后退
    if (safety_state_ != SafetyState::REVERSE) {
      safety_state_ = SafetyState::REVERSE;
      publishSafetyReverse(reverse_speed_);
      std_msgs::String state;
      state.data = "REVERSE";
      safety_state_pub_.publish(state);
      ROS_WARN("[distance_safety] ❌ 紧急后退！距离 %.0f mm < %.0f mm",
               current_distance_mm_, critical_distance_mm_);
    }
  } else if (current_distance_mm_ <= safety_distance_mm_) {
    // 安全停止
    if (safety_state_ != SafetyState::STOP) {
      safety_state_ = SafetyState::STOP;
      publishSafetyStop();
      std_msgs::String state;
      state.data = "STOP";
      safety_state_pub_.publish(state);
      ROS_WARN("[distance_safety] ⛔ 安全停止！距离 %.0f mm < %.0f mm",
               current_distance_mm_, safety_distance_mm_);

      // ── 激活 Bug2 避障（如果启用且有侧向传感器） ──
      if (bug2_enabled_ && has_odom_ && current_vx_ > 0.001) {
        target_yaw_ = current_yaw_;  // 记录当前目标方向
        bug2_state_ = Bug2State::STOPPED;
        bug2_start_time_ = ros::Time::now();
        ROS_INFO("[distance_safety] 🐛 Bug2 避障启动: 目标方向=%.2f rad", target_yaw_);
      }
    }
  } else if (current_distance_mm_ <= safety_distance_mm_ * 1.5) {
    // 接近警告：减速
    if (safety_state_ != SafetyState::WARNING) {
      safety_state_ = SafetyState::WARNING;
      // 发布减速指令（原速度的 30%）
      geometry_msgs::Twist slow;
      slow.linear.x = current_vx_ * 0.3;
      slow.linear.y = current_vy_ * 0.3;
      slow.angular.z = 0.0;
      cmd_vel_safety_pub_.publish(slow);
      std_msgs::String state;
      state.data = "WARNING";
      safety_state_pub_.publish(state);
    }
  } else {
    // 安全，清除干预
    if (safety_state_ != SafetyState::CLEAR) {
      safety_state_ = SafetyState::CLEAR;
      publishSafetyClear();
      std_msgs::String state;
      state.data = "CLEAR";
      safety_state_pub_.publish(state);

      // 障碍清除，Bug2 回归正常
      if (bug2_state_ != Bug2State::IDLE) {
        bug2_state_ = Bug2State::IDLE;
        ROS_INFO("[distance_safety] 🐛 Bug2 避障完成（障碍已清除）");
        std_msgs::String bs;
        bs.data = "IDLE";
        bug2_state_pub_.publish(bs);
      }
    }
  }

  // ── Bug2 状态机（在安全状态下运行） ──
  if (bug2_enabled_ && safety_state_ == SafetyState::STOP) {
    bug2StateMachine();
  }
}

// ── 安全停止 ───────────────────────────────────────────────
void DistanceSafetyNode::publishSafetyStop() {
  geometry_msgs::Twist stop;
  stop.linear.x = 0.0;
  stop.linear.y = 0.0;
  stop.angular.z = 0.0;
  cmd_vel_safety_pub_.publish(stop);
}

// ── 安全后退 ───────────────────────────────────────────────
void DistanceSafetyNode::publishSafetyReverse(double reverse_speed) {
  geometry_msgs::Twist reverse;
  reverse.linear.x = -std::abs(reverse_speed);
  reverse.linear.y = 0.0;
  reverse.angular.z = 0.0;
  cmd_vel_safety_pub_.publish(reverse);
}

// ── 清除安全干预 ───────────────────────────────────────────
void DistanceSafetyNode::publishSafetyClear() {
  // 发布一个"空"Twist，让 mux 知道 safety 通道无有效指令
  // mux 的 sourceActive 依赖超时判断 — 发布零速度让 mux 知道 safety 通道存在
  // 但不要在 safety CLEAR 时持续发布零速度覆盖其他通道！
  // 解决方案：发布一次速度，然后依赖 mux 的 cmd_timeout_sec 超时回退
  geometry_msgs::Twist clear;
  clear.linear.x = 0.0;
  clear.linear.y = 0.0;
  clear.angular.z = 0.0;
  // 仅发布一次就够了 — mux 的 sourceActive 检查 has_msg 和时间戳
  // 发布零速度后，mux 会切换到其他通道
  cmd_vel_safety_pub_.publish(clear);
}

double DistanceSafetyNode::clamp(double val, double lo, double hi) const {
  return std::max(lo, std::min(hi, val));
}

// ── Bug2 避障命令发布 ──────────────────────────────────────
void DistanceSafetyNode::publishBug2Command(double vx, double vy, double vw) {
  geometry_msgs::Twist cmd;
  cmd.linear.x = vx;
  cmd.linear.y = vy;
  cmd.angular.z = vw;
  cmd_vel_safety_pub_.publish(cmd);
}

// ── Bug2 避障状态机 ────────────────────────────────────────
void DistanceSafetyNode::bug2StateMachine() {
  ros::Time now = ros::Time::now();
  double bug2_elapsed = (now - bug2_start_time_).toSec();

  // 超时保护
  if (bug2_elapsed > bug2_max_duration_sec_) {
    ROS_WARN("[distance_safety] 🐛 Bug2 避障超时 (%.1f s)，放弃绕行", bug2_elapsed);
    bug2_state_ = Bug2State::FAILED;
    std_msgs::String bs;
    bs.data = "FAILED";
    bug2_state_pub_.publish(bs);
    publishSafetyClear();  // 恢复让上层决策
    return;
  }

  switch (bug2_state_) {
    case Bug2State::STOPPED: {
      // 等待1秒稳定后检查侧向空间
      if (bug2_elapsed < 0.5) break;
      bug2_state_ = Bug2State::CHECKING_SIDES;
      ROS_INFO("[distance_safety] 🐛 检查侧向空间: L=%.0f mm, R=%.0f mm",
               current_distance_left_mm_, current_distance_right_mm_);
      std_msgs::String bs;
      bs.data = "CHECKING_SIDES";
      bug2_state_pub_.publish(bs);
      break;
    }

    case Bug2State::CHECKING_SIDES: {
      // 优先选择空间更大的一侧
      double left = has_distance_left_ ? current_distance_left_mm_ : bug2_side_clearance_mm_;
      double right = has_distance_right_ ? current_distance_right_mm_ : bug2_side_clearance_mm_;

      if (left < bug2_side_clearance_mm_ && right < bug2_side_clearance_mm_) {
        // 两侧都不够，后退后再试
        ROS_WARN("[distance_safety] 🐛 两侧空间不足 (L:%.0f, R:%.0f)，尝试后退",
                 left, right);
        bug2_state_ = Bug2State::FAILED;
        publishSafetyReverse(reverse_speed_);
        break;
      }

      // 转向空间更大的一侧
      bug2_turn_direction_ = (right >= left);
      bug2_state_ = Bug2State::TURNING;
      ROS_INFO("[distance_safety] 🐛 转向%s侧", bug2_turn_direction_ ? "右" : "左");
      std_msgs::String bs;
      bs.data = "TURNING";
      bug2_state_pub_.publish(bs);
      break;
    }

    case Bug2State::TURNING: {
      // 原地旋转 ~90°
      double turn_angle = bug2_turn_direction_ ? -bug2_turn_speed_ : bug2_turn_speed_;
      publishBug2Command(0.0, 0.0, turn_angle);

      // 前方距离恢复后跳出转弯
      if (current_distance_mm_ > safety_distance_mm_ * 1.2) {
        bug2_state_ = Bug2State::MOVING_ALONG;
        ROS_INFO("[distance_safety] 🐛 前方已通畅，沿障碍物边缘前进");
        std_msgs::String bs;
        bs.data = "MOVING_ALONG";
        bug2_state_pub_.publish(bs);
      }
      break;
    }

    case Bug2State::MOVING_ALONG: {
      // 沿当前方向前进
      publishBug2Command(bug2_along_speed_, 0.0, 0.0);

      // 计算与目标方向的偏差
      if (has_odom_) {
        double yaw_diff = std::abs(current_yaw_ - target_yaw_);
        // 规范化角度差
        while (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;

        // 如果前方再次出现障碍，重新停止
        if (current_distance_mm_ <= safety_distance_mm_) {
          bug2_state_ = Bug2State::STOPPED;
          bug2_start_time_ = ros::Time::now();
          ROS_INFO("[distance_safety] 🐛 再次遇到障碍，重新评估");
          break;
        }

        // 前进一段距离后尝试回归原方向
        if (bug2_elapsed > 3.0 && yaw_diff > bug2_goal_tolerance_rad_) {
          bug2_state_ = Bug2State::BACK_TO_PATH;
          ROS_INFO("[distance_safety] 🐛 尝试回归原始方向 (偏差: %.2f rad)", yaw_diff);
          std_msgs::String bs;
          bs.data = "BACK_TO_PATH";
          bug2_state_pub_.publish(bs);
        }
      }
      break;
    }

    case Bug2State::BACK_TO_PATH: {
      // 回归目标方向
      double yaw_diff = target_yaw_ - current_yaw_;
      while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
      while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;

      if (std::abs(yaw_diff) < bug2_goal_tolerance_rad_) {
        // 已回归目标方向，避障完成
        ROS_INFO("[distance_safety] 🐛 Bug2 避障完成！已回归目标方向");
        bug2_state_ = Bug2State::IDLE;
        publishSafetyClear();
        std_msgs::String bs;
        bs.data = "IDLE";
        bug2_state_pub_.publish(bs);
        break;
      }

      // 转向目标方向
      double turn_sign = (yaw_diff > 0) ? 1.0 : -1.0;
      double turn_cmd = turn_sign * std::min(bug2_turn_speed_, std::abs(yaw_diff));
      publishBug2Command(bug2_along_speed_ * 0.5, 0.0, turn_cmd);

      // 安全：前方又出现障碍则重新绕过
      if (current_distance_mm_ <= safety_distance_mm_) {
        bug2_state_ = Bug2State::STOPPED;
        bug2_start_time_ = ros::Time::now();
        ROS_INFO("[distance_safety] 🐛 回归途中再次遇到障碍");
      }
      break;
    }

    case Bug2State::FAILED:
    case Bug2State::IDLE:
    default:
      break;
  }
}

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "distance_safety_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::DistanceSafetyNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[distance_safety] 初始化失败");
    return 1;
  }

  ROS_INFO("[distance_safety] 距离安全检测节点运行中...");
  ros::spin();

  return 0;
}
