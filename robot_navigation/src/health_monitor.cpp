/**
 * @file health_monitor.cpp
 * @brief 系统健康监控节点实现
 */

#include "robot_navigation/health_monitor.h"

#include <sstream>
#include <algorithm>

namespace robot_navigation {

// ── 构造 ───────────────────────────────────────────────────
HealthMonitor::HealthMonitor(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , publish_rate_hz_(1.0)
    , system_ok_(true)
    , consecutive_failures_(0)
    , max_consecutive_failures_(3)
{
  pnh_.param<double>("publish_rate_hz", publish_rate_hz_, 1.0);
  pnh_.param<int>("max_consecutive_failures", max_consecutive_failures_, 3);

  // 默认监控的关键话题
  critical_topics_ = {
    "/odom",
    "/vl53l1x_distance",
    "/motor_state",
    "/path_points"
  };

  // 允许从参数覆盖
  pnh_.getParam("critical_topics", critical_topics_);

  for (const auto& topic : critical_topics_) {
    TopicStatus ts;
    ts.seen = false;
    ts.last_seen = ros::Time::now();
    ts.timeout_sec = 2.0;  // 2秒内至少收到一次
    topic_status_[topic] = ts;
  }
}

// ── 初始化 ─────────────────────────────────────────────────
bool HealthMonitor::init() {
  stop_all_sub_ = nh_.subscribe("/stop_all", 10,
                                &HealthMonitor::stopAllCallback, this);

  health_pub_ = nh_.advertise<std_msgs::String>("/system_health", 10, true);
  health_ok_pub_ = nh_.advertise<std_msgs::Bool>("/system_health/ok", 10, true);
  emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 10, true);

  reset_srv_ = nh_.advertiseService("/system_health/reset",
                                    &HealthMonitor::resetCallback, this);

  const double period = 1.0 / std::max(0.1, publish_rate_hz_);
  health_timer_ = nh_.createTimer(ros::Duration(period),
                                  &HealthMonitor::healthTimerCallback, this);

  // 发布初始状态
  {
    std_msgs::String init;
    init.data = "系统启动中...";
    health_pub_.publish(init);

    std_msgs::Bool ok;
    ok.data = false;
    health_ok_pub_.publish(ok);
  }

  ROS_INFO("[health_monitor] 初始化完成");
  ROS_INFO("[health_monitor]   发布频率: %.1f Hz", publish_rate_hz_);
  ROS_INFO("[health_monitor]   监控话题数: %zu", critical_topics_.size());
  for (const auto& t : critical_topics_) {
    ROS_INFO("[health_monitor]     - %s", t.c_str());
  }

  return true;
}

// ── /stop_all 回调（转发紧急停止） ─────────────────────────
void HealthMonitor::stopAllCallback(const std_msgs::Empty::ConstPtr& /*msg*/) {
  ROS_WARN("[health_monitor] 收到 /stop_all 信号，触发紧急停止");

  std_msgs::Bool estop;
  estop.data = true;
  emergency_stop_pub_.publish(estop);

  // 延迟后清除紧急停止（让 mux 有机会处理）
  ros::Duration(0.1).sleep();

  // 不清除 — 由 /system_health/reset 手动清除
}

// ── 健康检查定时器 ─────────────────────────────────────────
void HealthMonitor::healthTimerCallback(const ros::TimerEvent& /*event*/) {
  checkCriticalTopics();

  std::string report = generateHealthReport();

  std_msgs::String msg;
  msg.data = report;
  health_pub_.publish(msg);

  bool was_ok = system_ok_;
  system_ok_ = true;

  for (const auto& pair : topic_status_) {
    if (!pair.second.seen) {
      system_ok_ = false;
      break;
    }
  }

  if (!system_ok_) {
    consecutive_failures_++;
    if (consecutive_failures_ >= max_consecutive_failures_) {
      ROS_ERROR_THROTTLE(5.0, "[health_monitor] ⚠ 系统异常！%d 次连续故障",
                         consecutive_failures_);
    }
  } else {
    consecutive_failures_ = 0;
  }

  if (system_ok_ != was_ok) {
    std_msgs::Bool ok;
    ok.data = system_ok_;
    health_ok_pub_.publish(ok);

    if (system_ok_) {
      ROS_INFO("[health_monitor] ✅ 系统恢复正常");
    } else {
      ROS_WARN("[health_monitor] ❌ 系统状态异常");
    }
  }
}

// ── 检查关键话题活跃状态 ──────────────────────────────────
void HealthMonitor::checkCriticalTopics() {
  ros::Time now = ros::Time::now();

  for (auto& pair : topic_status_) {
    TopicStatus& ts = pair.second;

    // 统计该话题的发布者数量
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);

    bool has_publisher = false;
    for (const auto& t : topics) {
      if (t.name == pair.first && !t.datatype.empty()) {
        has_publisher = true;
        break;
      }
    }

    if (!has_publisher) {
      ts.seen = false;
      continue;
    }

    // 话题有发布者 → 认为活跃（简化判定）
    // 更精确的方式是订阅并检查数据时间戳，但这会增加复杂度
    ts.seen = true;
    ts.last_seen = now;
  }
}

// ── 生成健康报告 ───────────────────────────────────────────
std::string HealthMonitor::generateHealthReport() const {
  std::ostringstream ss;
  ss << "=== 系统健康状态 ===\n";

  int ok_count = 0;
  int fail_count = 0;

  for (const auto& pair : topic_status_) {
    const TopicStatus& ts = pair.second;
    if (ts.seen) {
      ss << "  ✅ " << pair.first << "\n";
      ok_count++;
    } else {
      ss << "  ❌ " << pair.first << " (无发布者)\n";
      fail_count++;
    }
  }

  ss << "---\n";
  ss << "  正常: " << ok_count << "/" << (ok_count + fail_count);
  if (consecutive_failures_ > 0) {
    ss << " | 连续故障: " << consecutive_failures_;
  }
  ss << "\n";

  return ss.str();
}

// ── 重置服务 ───────────────────────────────────────────────
bool HealthMonitor::resetCallback(std_srvs::Trigger::Request& /*req*/,
                                  std_srvs::Trigger::Response& res) {
  ROS_INFO("[health_monitor] 收到重置请求");

  consecutive_failures_ = 0;
  for (auto& pair : topic_status_) {
    pair.second.seen = false;
    pair.second.last_seen = ros::Time::now();
  }

  // 清除紧急停止
  std_msgs::Bool estop;
  estop.data = false;
  emergency_stop_pub_.publish(estop);

  res.success = true;
  res.message = "系统监控已重置，紧急停止已解除";
  return true;
}

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "health_monitor_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::HealthMonitor monitor(nh, pnh);
  if (!monitor.init()) {
    ROS_FATAL("[health_monitor] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
