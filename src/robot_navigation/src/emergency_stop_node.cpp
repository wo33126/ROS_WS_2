/**
 * @file emergency_stop_node.cpp
 * @brief 急停按钮节点
 *
 * 监听急停按钮 GPIO，提供双重保护：
 *   1. 软件层：检测到急停时立即发布 /emergency_stop 话题
 *      cmd_vel_mux 收到后强制输出零速度
 *   2. 硬件建议：急停按钮（常闭触点）串联到电机电源继电器
 *
 * 接线（常闭触点）：
 *   急停按钮 NC → GPIO（BCM 22，物理引脚 15）
 *   急停按钮 COM → GND
 *   内部上拉启用 — 正常时读高电平，急停时读低电平
 *
 * 行为：
 *   - 按下急停：发布 /emergency_stop = true，机器人立即停止
 *   - 松开急停：发布 /emergency_stop = false，需重新按启动按钮
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <fstream>
#include <string>
#include <cstring>
#include <cerrno>

namespace robot_navigation {

/**
 * @brief sysfs GPIO 读取器（轻量级实现）
 */
class SysfsGpio {
 public:
  bool exportPin(int pin, const std::string& direction = "in") {
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) return false;
    export_file << pin;
    export_file.close();
    usleep(100000);

    base_path_ = "/sys/class/gpio/gpio" + std::to_string(pin);

    std::ofstream dir_file(base_path_ + "/direction");
    if (dir_file.is_open()) {
      dir_file << direction;
      dir_file.close();
    }
    pin_ = pin;
    return true;
  }

  int read() {
    std::ifstream val_file(base_path_ + "/value");
    if (!val_file.is_open()) return -1;
    char c;
    val_file >> c;
    return (c == '1') ? 1 : 0;
  }

  bool write(int value) {
    std::ofstream val_file(base_path_ + "/value");
    if (!val_file.is_open()) return false;
    val_file << (value ? "1" : "0");
    return true;
  }

  void unexport() {
    std::ofstream unexport_file("/sys/class/gpio/unexport");
    if (unexport_file.is_open()) {
      unexport_file << pin_;
      unexport_file.close();
    }
  }

 private:
  int pin_ = -1;
  std::string base_path_;
};

// ==========================================================================
//  EmergencyStopNode
// ==========================================================================

class EmergencyStopNode {
 public:
  EmergencyStopNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , estop_pin_(22)       // BCM 22 (物理引脚 15)
      , led_pin_(23)         // BCM 23 — 急停指示灯
      , debounce_ms_(30)     // 急停用更短的去抖时间
      , active_low_(true)    // 常闭触点，正常高，急停低
      , poll_rate_hz_(100.0) // 高频轮询确保反应迅速
      , estop_active_(false)
      , running_(false)
  {
    pnh_.param<int>("estop_pin", estop_pin_, 22);
    pnh_.param<int>("led_pin", led_pin_, 23);
    pnh_.param<int>("debounce_ms", debounce_ms_, 30);
    pnh_.param<bool>("active_low", active_low_, true);
    pnh_.param<double>("poll_rate_hz", poll_rate_hz_, 100.0);
  }

  ~EmergencyStopNode() {
    stop();
  }

  bool init() {
    // 导出急停引脚
    if (!estop_gpio_.exportPin(estop_pin_, "in")) {
      ROS_ERROR("[emergency_stop] 无法导出急停 GPIO %d", estop_pin_);
      return false;
    }

    // 导出 LED 引脚
    led_gpio_.exportPin(led_pin_, "out");

    // 发布者
    estop_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 1, true);
    estop_state_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop/state", 1, true);
    estop_event_pub_ = nh_.advertise<std_msgs::Empty>("/emergency_stop/event", 1, true);

    // 立即发布初始状态（正常）
    publishEstopState(false);

    // 启动轮询线程
    running_ = true;
    poll_thread_ = std::thread(&EmergencyStopNode::pollLoop, this);

    ROS_INFO("[emergency_stop] 初始化完成");
    ROS_INFO("[emergency_stop]   急停按钮: GPIO %d, 去抖: %d ms, 电平: %s",
             estop_pin_, debounce_ms_, active_low_ ? "低有效(常闭)" : "高有效(常开)");
    ROS_INFO("[emergency_stop]   LED指示: GPIO %d", led_pin_);
    ROS_INFO("[emergency_stop]   发布话题: /emergency_stop (latch=true)");
    ROS_INFO("[emergency_stop]   监控中...");

    return true;
  }

  void stop() {
    running_ = false;
    if (poll_thread_.joinable()) poll_thread_.join();
    estop_gpio_.unexport();
    led_gpio_.unexport();
  }

 private:
  void pollLoop() {
    const int poll_us = static_cast<int>(1000000.0 / std::max(1.0, poll_rate_hz_));

    // 去抖动状态机
    enum class DebounceState { IDLE, TRIGGER_PENDING, TRIGGERED, RELEASE_PENDING };
    DebounceState state = DebounceState::IDLE;
    auto state_enter_time = std::chrono::steady_clock::now();

    while (running_ && ros::ok()) {
      int raw = estop_gpio_.read();
      // active_low_: 正常=高, 急停=低
      bool triggered = active_low_ ? (raw == 0) : (raw == 1);

      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - state_enter_time).count();

      switch (state) {
        case DebounceState::IDLE:
          if (triggered) {
            state = DebounceState::TRIGGER_PENDING;
            state_enter_time = now;
          }
          break;

        case DebounceState::TRIGGER_PENDING:
          if (!triggered) {
            state = DebounceState::IDLE;
          } else if (elapsed >= debounce_ms_) {
            state = DebounceState::TRIGGERED;
            onEstopActivated();
          }
          break;

        case DebounceState::TRIGGERED:
          if (!triggered) {
            state = DebounceState::RELEASE_PENDING;
            state_enter_time = now;
          }
          break;

        case DebounceState::RELEASE_PENDING:
          if (triggered) {
            state = DebounceState::TRIGGERED;
          } else if (elapsed >= debounce_ms_) {
            state = DebounceState::IDLE;
            onEstopReleased();
          }
          break;
      }

      usleep(poll_us);
    }
  }

  void onEstopActivated() {
    estop_active_ = true;
    ROS_ERROR("[emergency_stop] ====== 🛑 紧急停止！======");

    publishEstopState(true);

    // 发布急停事件
    std_msgs::Empty event;
    estop_event_pub_.publish(event);

    // LED 快闪
    led_gpio_.write(0);
  }

  void onEstopReleased() {
    estop_active_ = false;
    ROS_WARN("[emergency_stop] 急停已释放，等待重新启动");

    publishEstopState(false);

    // LED 恢复常亮
    led_gpio_.write(1);
  }

  void publishEstopState(bool active) {
    std_msgs::Bool msg;
    msg.data = active;
    estop_pub_.publish(msg);
    estop_state_pub_.publish(msg);
  }

  // ── ROS ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher estop_pub_;
  ros::Publisher estop_state_pub_;
  ros::Publisher estop_event_pub_;

  // ── GPIO ──
  SysfsGpio estop_gpio_;
  SysfsGpio led_gpio_;

  // ── 配置 ──
  int estop_pin_;
  int led_pin_;
  int debounce_ms_;
  bool active_low_;
  double poll_rate_hz_;

  // ── 状态 ──
  std::atomic<bool> estop_active_;
  std::atomic<bool> running_;
  std::thread poll_thread_;
};

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "emergency_stop_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::EmergencyStopNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[emergency_stop] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
