/**
 * @file start_button_node.cpp
 * @brief 物理启动按钮节点 — GPIO 监听
 *
 * 监听 Raspberry Pi GPIO 引脚上的物理按键：
 *   - 去抖动处理（50ms）
 *   - 上升沿/下降沿检测
 *   - 发布 std_msgs/Empty 到 /start_signal
 *   - 可选：LED 指示灯控制（GPIO 输出）
 *
 * 硬件接线：
 *   按钮一端 → GPIO（BCM 17，物理引脚 11）
 *   按钮另一端 → GND
 *   内部上拉电阻启用（按下时 GPIO 读低电平）
 *   LED(+) → GPIO（BCM 27）→ 220Ω → GND
 *
 * 依赖: pigpio 库（sudo apt install pigpio, sudo pigpiod）
 *       或使用 sysfs GPIO（无需额外依赖）
 */

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <atomic>
#include <chrono>
#include <thread>
#include <cstring>
#include <string>
#include <fstream>
#include <cerrno>

namespace robot_navigation {

/**
 * @brief 使用 sysfs 接口的 GPIO 读取器（无需外部库）
 */
class SysfsGpio {
 public:
  SysfsGpio() = default;

  /**
   * @brief 导出 GPIO 引脚
   * @param pin BCM 编号
   * @param direction "in" 或 "out"
   * @param edge "rising", "falling", "both", "none"
   */
  bool exportPin(int pin, const std::string& direction = "in",
                 const std::string& edge = "both") {
    // 导出
    std::ofstream export_file("/sys/class/gpio/export");
    if (!export_file.is_open()) {
      ROS_ERROR("[start_button] 无法访问 /sys/class/gpio/export (需要 root 或 gpio 组权限)");
      return false;
    }
    export_file << pin;
    export_file.close();

    // 等待 sysfs 创建目录
    usleep(100000);

    std::string base = "/sys/class/gpio/gpio" + std::to_string(pin);

    // 设置方向
    std::ofstream dir_file(base + "/direction");
    if (!dir_file.is_open()) {
      // 可能已经导出过
      ROS_WARN("[start_button] GPIO %d 可能已导出", pin);
    } else {
      dir_file << direction;
      dir_file.close();
    }

    // 设置边沿检测
    std::ofstream edge_file(base + "/edge");
    if (edge_file.is_open()) {
      edge_file << edge;
      edge_file.close();
    }

    pin_ = pin;
    base_path_ = base;
    return true;
  }

  /**
   * @brief 读取 GPIO 值
   * @return 0 或 1, -1 表示错误
   */
  int read() {
    std::ifstream val_file(base_path_ + "/value");
    if (!val_file.is_open()) return -1;
    char c;
    val_file >> c;
    return (c == '1') ? 1 : 0;
  }

  /**
   * @brief 写入 GPIO 值（仅输出模式）
   */
  bool write(int value) {
    std::ofstream val_file(base_path_ + "/value");
    if (!val_file.is_open()) return false;
    val_file << (value ? "1" : "0");
    return true;
  }

  /**
   * @brief 释放 GPIO
   */
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
//  StartButtonNode
// ==========================================================================

class StartButtonNode {
 public:
  StartButtonNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , button_pin_(17)       // BCM 17 (物理引脚 11)
      , led_pin_(27)          // BCM 27 (物理引脚 13)
      , debounce_ms_(50)
      , active_low_(true)     // 按下接地，读低电平
      , led_enabled_(true)
      , poll_rate_hz_(50.0)
      , button_pressed_(false)
      , running_(false)
  {
    pnh_.param<int>("button_pin", button_pin_, 17);
    pnh_.param<int>("led_pin", led_pin_, 27);
    pnh_.param<int>("debounce_ms", debounce_ms_, 50);
    pnh_.param<bool>("active_low", active_low_, true);
    pnh_.param<bool>("led_enabled", led_enabled_, true);
    pnh_.param<double>("poll_rate_hz", poll_rate_hz_, 50.0);
  }

  ~StartButtonNode() {
    stop();
  }

  bool init() {
    // 导出按钮引脚
    if (!button_gpio_.exportPin(button_pin_, "in", "both")) {
      ROS_WARN("[start_button] GPIO 按钮初始化失败，将仅依赖 /start_signal_manual 话题");
    } else {
      ROS_INFO("[start_button] 按钮 GPIO %d 已配置 (active_low=%s)",
               button_pin_, active_low_ ? "true" : "false");
    }

    // 导出 LED 引脚
    if (led_enabled_) {
      if (led_gpio_.exportPin(led_pin_, "out", "none")) {
        led_gpio_.write(1);  // 初始点亮 LED（表示就绪）
        ROS_INFO("[start_button] LED GPIO %d 已配置", led_pin_);
      }
    }

    // 发布者
    start_signal_pub_ = nh_.advertise<std_msgs::Empty>("/start_signal", 1, true);
    led_state_pub_ = nh_.advertise<std_msgs::Bool>("/start_button/led", 1, true);

    // 备用：也发布到 /start_signal_manual 供调试
    manual_signal_pub_ = nh_.advertise<std_msgs::Empty>("/start_signal_manual", 1, true);

    // 启动轮询线程
    running_ = true;
    poll_thread_ = std::thread(&StartButtonNode::pollLoop, this);

    ROS_INFO("[start_button] 初始化完成");
    ROS_INFO("[start_button]   按钮: GPIO %d, 去抖: %d ms, 电平: %s",
             button_pin_, debounce_ms_, active_low_ ? "低有效" : "高有效");
    ROS_INFO("[start_button]   发布话题: /start_signal (latch=true)");
    ROS_INFO("[start_button]   等待按钮按下...");

    return true;
  }

  void stop() {
    running_ = false;
    if (poll_thread_.joinable()) poll_thread_.join();
    button_gpio_.unexport();
    if (led_enabled_) led_gpio_.unexport();
  }

 private:
  void pollLoop() {
    const int poll_us = static_cast<int>(1000000.0 / std::max(1.0, poll_rate_hz_));

    // 去抖动状态机
    enum class DebounceState { IDLE, PRESS_PENDING, PRESSED, RELEASE_PENDING };
    DebounceState state = DebounceState::IDLE;
    auto state_enter_time = std::chrono::steady_clock::now();

    while (running_ && ros::ok()) {
      int raw = button_gpio_.read();
      bool level = active_low_ ? (raw == 0) : (raw == 1);

      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - state_enter_time).count();

      switch (state) {
        case DebounceState::IDLE:
          if (level) {
            state = DebounceState::PRESS_PENDING;
            state_enter_time = now;
          }
          break;

        case DebounceState::PRESS_PENDING:
          if (!level) {
            state = DebounceState::IDLE;  // 噪声，回到空闲
          } else if (elapsed >= debounce_ms_) {
            // 确认按下
            state = DebounceState::PRESSED;
            onButtonPressed();
          }
          break;

        case DebounceState::PRESSED:
          if (!level) {
            state = DebounceState::RELEASE_PENDING;
            state_enter_time = now;
          }
          break;

        case DebounceState::RELEASE_PENDING:
          if (level) {
            state = DebounceState::PRESSED;  // 噪声
          } else if (elapsed >= debounce_ms_) {
            // 确认释放
            state = DebounceState::IDLE;
          }
          break;
      }

      usleep(poll_us);
    }
  }

  void onButtonPressed() {
    ROS_INFO("[start_button] ====== 物理按钮按下！发送启动信号 ======");

    // 发布启动信号
    std_msgs::Empty signal;
    start_signal_pub_.publish(signal);
    manual_signal_pub_.publish(signal);

    // LED 闪烁指示
    if (led_enabled_) {
      led_gpio_.write(0);  // 灭灯表示已触发
      std_msgs::Bool led;
      led.data = false;
      led_state_pub_.publish(led);

      // 1 秒后重新点亮
      std::thread([this]() {
        usleep(1000000);
        if (led_enabled_) {
          led_gpio_.write(1);
          std_msgs::Bool led;
          led.data = true;
          led_state_pub_.publish(led);
        }
      }).detach();
    }

    ROS_INFO("[start_button] 启动信号已发布");
  }

  // ── ROS ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher start_signal_pub_;
  ros::Publisher manual_signal_pub_;
  ros::Publisher led_state_pub_;

  // ── GPIO ──
  SysfsGpio button_gpio_;
  SysfsGpio led_gpio_;

  // ── 配置 ──
  int button_pin_;
  int led_pin_;
  int debounce_ms_;
  bool active_low_;
  bool led_enabled_;
  double poll_rate_hz_;

  // ── 状态 ──
  std::atomic<bool> button_pressed_;
  std::atomic<bool> running_;
  std::thread poll_thread_;
};

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "start_button_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::StartButtonNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[start_button] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
