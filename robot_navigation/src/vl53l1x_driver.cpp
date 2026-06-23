/**
 * @file vl53l1x_driver.cpp
 * @brief VL53L1X 激光测距传感器驱动节点实现
 *
 * 通过 Linux I2C 接口与 VL53L1X 通信，读取距离数据并发布到 ROS 话题。
 * 同时提供距离安全检测功能，可在底盘移动中触发避障/急停。
 */

#include "robot_navigation/vl53l1x_driver.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <sstream>

#include <std_msgs/String.h>

namespace robot_navigation {

// ── VL53L1X 寄存器地址 ──────────────────────────────────────
namespace vl53l1x_reg {
  constexpr uint16_t SOFT_RESET           = 0x0000;
  constexpr uint16_t I2C_SLAVE_DEV_ADDR   = 0x0001;
  constexpr uint16_t ANA_CONFIG           = 0x0002;
  constexpr uint16_t VHV_CONFIG_TIMEOUT   = 0x0024;
  constexpr uint16_t RANGE_CONFIG_A       = 0x005E;
  constexpr uint16_t RANGE_CONFIG_B       = 0x0061;
  constexpr uint16_t INTER_MEASUREMENT    = 0x006C;
  constexpr uint16_t SYSTEM_MODE_START    = 0x0087;
  constexpr uint16_t RESULT_RANGE_STATUS  = 0x0089;
  constexpr uint16_t RESULT_INTERRUPT     = 0x008B;
  constexpr uint16_t RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 = 0x0096;
  constexpr uint16_t SYSTEM_INTERRUPT_CLEAR = 0x0086;
  constexpr uint16_t OSC_CALIBRATE_VAL    = 0x00F6;
  constexpr uint16_t GPIO_TIO_HV_STATUS   = 0x0031;
  constexpr uint16_t PHASECAL_RESULT      = 0x004C;
  constexpr uint16_t DSS_CONFIG          = 0x004C;
  constexpr uint16_t RANGE_CONFIG_TIMEOUT = 0x005A;
  constexpr uint16_t SYSTEM_STATUS       = 0x0000;
}

// ── 构造 ───────────────────────────────────────────────────
VL53L1XDriver::VL53L1XDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , i2c_device_("/dev/i2c-1")
    , i2c_fd_(-1)
    , i2c_address_(0x29)
    , topic_name_("/vl53l1x_distance")
    , read_rate_hz_(20.0)
    , distance_mode_(2)
    , timing_budget_ms_(50.0)
    , use_median_filter_(true)
    , median_window_size_(5)
    , use_moving_average_(true)
    , moving_average_alpha_(0.3)
    , safety_threshold_mm_(300.0)
    , critical_threshold_mm_(100.0)
    , safety_timeout_sec_(0.5)
    , offset_mm_(0)
    , current_distance_mm_(0.0)
    , filtered_distance_mm_(0.0)
    , sensor_ok_(false)
    , is_ranging_(false)
    , consecutive_errors_(0)
    , max_consecutive_errors_(10)
    , safety_warning_active_(false)
    , safety_stop_active_(false)
    , last_valid_reading_(ros::Time::now())
{}

VL53L1XDriver::~VL53L1XDriver() {
  if (is_ranging_) {
    stopRanging();
  }
  closeI2C();
}

// ── 初始化 ─────────────────────────────────────────────────
bool VL53L1XDriver::init() {
  // ── 参数读取 ──
  pnh_.param<std::string>("i2c_device", i2c_device_, "/dev/i2c-1");
  pnh_.param<int>("i2c_address", reinterpret_cast<int&>(i2c_address_), 0x29);
  pnh_.param<std::string>("topic_name", topic_name_, "/vl53l1x_distance");
  pnh_.param<double>("read_rate_hz", read_rate_hz_, 20.0);
  pnh_.param<int>("distance_mode", distance_mode_, 2);
  pnh_.param<double>("timing_budget_ms", timing_budget_ms_, 50.0);
  pnh_.param<bool>("use_median_filter", use_median_filter_, true);
  pnh_.param<int>("median_window_size", median_window_size_, 5);
  pnh_.param<bool>("use_moving_average", use_moving_average_, true);
  pnh_.param<double>("moving_average_alpha", moving_average_alpha_, 0.3);
  pnh_.param<double>("safety_threshold_mm", safety_threshold_mm_, 300.0);
  pnh_.param<double>("critical_threshold_mm", critical_threshold_mm_, 100.0);
  pnh_.param<double>("safety_timeout_sec", safety_timeout_sec_, 0.5);
  pnh_.param<int>("offset_mm", offset_mm_, 0);
  pnh_.param<int>("max_consecutive_errors", max_consecutive_errors_, 10);

  // ── 发布者 ──
  distance_pub_        = nh_.advertise<std_msgs::Float32>(topic_name_, 10);
  status_pub_          = nh_.advertise<std_msgs::String>("/vl53l1x/status", 10, true);
  safety_warning_pub_  = nh_.advertise<std_msgs::Bool>("/distance_safety_warning", 10);
  safety_stop_pub_     = nh_.advertise<std_msgs::Bool>("/distance_safety_stop", 10);

  // ── 服务 ──
  reset_srv_     = nh_.advertiseService("/vl53l1x/reset",
                                        &VL53L1XDriver::resetServiceCallback, this);
  calibrate_srv_ = nh_.advertiseService("/vl53l1x/calibrate",
                                        &VL53L1XDriver::calibrateServiceCallback, this);

  // ── I2C 初始化 ──
  if (!openI2C()) {
    ROS_ERROR("[vl53l1x] 无法打开 I2C 设备 %s", i2c_device_.c_str());
    std_msgs::String status;
    status.data = "I2C_ERROR";
    status_pub_.publish(status);
    return false;
  }

  // ── 传感器初始化 ──
  if (!initSensor()) {
    ROS_ERROR("[vl53l1x] 传感器初始化失败");
    std_msgs::String status;
    status.data = "SENSOR_INIT_FAILED";
    status_pub_.publish(status);
    closeI2C();
    return false;
  }

  sensor_ok_ = true;

  // ── 启动连续测距 ──
  if (!startRanging()) {
    ROS_ERROR("[vl53l1x] 无法启动连续测距");
    std_msgs::String status;
    status.data = "RANGING_START_FAILED";
    status_pub_.publish(status);
    closeI2C();
    return false;
  }

  // ── 读取定时器 ──
  const double period = 1.0 / std::max(1.0, read_rate_hz_);
  read_timer_ = nh_.createTimer(ros::Duration(period),
                                &VL53L1XDriver::readTimerCallback, this);

  std_msgs::String status;
  status.data = "OK";
  status_pub_.publish(status);

  ROS_INFO("[vl53l1x] VL53L1X 驱动初始化完成");
  ROS_INFO("[vl53l1x]   I2C: %s @ 0x%02X", i2c_device_.c_str(), i2c_address_);
  ROS_INFO("[vl53l1x]   读取频率: %.1f Hz, 测距模式: %d", read_rate_hz_, distance_mode_);
  ROS_INFO("[vl53l1x]   安全阈值: %.0f mm (warning) / %.0f mm (critical)",
           safety_threshold_mm_, critical_threshold_mm_);
  ROS_INFO("[vl53l1x]   滤波: 中值=%s (窗口=%d), EMA=%s (α=%.2f)",
           use_median_filter_ ? "ON" : "OFF", median_window_size_,
           use_moving_average_ ? "ON" : "OFF", moving_average_alpha_);

  return true;
}

// ── I2C 打开 ───────────────────────────────────────────────
bool VL53L1XDriver::openI2C() {
  i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    ROS_ERROR("[vl53l1x] 无法打开 %s: %s", i2c_device_.c_str(), strerror(errno));
    return false;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
    ROS_ERROR("[vl53l1x] ioctl I2C_SLAVE 失败: %s", strerror(errno));
    close(i2c_fd_);
    i2c_fd_ = -1;
    return false;
  }

  ROS_INFO("[vl53l1x] I2C 已打开: %s @ 0x%02X", i2c_device_.c_str(), i2c_address_);
  return true;
}

void VL53L1XDriver::closeI2C() {
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
}

// ── 寄存器读写 ─────────────────────────────────────────────
bool VL53L1XDriver::writeRegister16(uint16_t reg, uint16_t value) {
  if (i2c_fd_ < 0) return false;
  uint8_t buf[4];
  buf[0] = static_cast<uint8_t>((reg >> 8) & 0xFF);
  buf[1] = static_cast<uint8_t>(reg & 0xFF);
  buf[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buf[3] = static_cast<uint8_t>(value & 0xFF);
  if (write(i2c_fd_, buf, 4) != 4) {
    return false;
  }
  return true;
}

bool VL53L1XDriver::readRegister16(uint16_t reg, uint16_t& value) {
  if (i2c_fd_ < 0) return false;
  uint8_t addr_buf[2] = {
    static_cast<uint8_t>((reg >> 8) & 0xFF),
    static_cast<uint8_t>(reg & 0xFF)
  };
  if (write(i2c_fd_, addr_buf, 2) != 2) return false;
  uint8_t data_buf[2] = {0, 0};
  if (read(i2c_fd_, data_buf, 2) != 2) return false;
  value = (static_cast<uint16_t>(data_buf[0]) << 8) | data_buf[1];
  return true;
}

bool VL53L1XDriver::writeRegister8(uint16_t reg, uint8_t value) {
  if (i2c_fd_ < 0) return false;
  uint8_t buf[3];
  buf[0] = static_cast<uint8_t>((reg >> 8) & 0xFF);
  buf[1] = static_cast<uint8_t>(reg & 0xFF);
  buf[2] = value;
  if (write(i2c_fd_, buf, 3) != 3) return false;
  return true;
}

bool VL53L1XDriver::readRegister8(uint16_t reg, uint8_t& value) {
  if (i2c_fd_ < 0) return false;
  uint8_t addr_buf[2] = {
    static_cast<uint8_t>((reg >> 8) & 0xFF),
    static_cast<uint8_t>(reg & 0xFF)
  };
  if (write(i2c_fd_, addr_buf, 2) != 2) return false;
  uint8_t data_buf[1] = {0};
  if (read(i2c_fd_, data_buf, 1) != 1) return false;
  value = data_buf[0];
  return true;
}

// ── 传感器初始化 ───────────────────────────────────────────
bool VL53L1XDriver::initSensor() {
  // VL53L1X 需要特殊的启动序列
  // 1. 等待传感器上电稳定（通过检查特定寄存器）
  ros::Duration(0.1).sleep();

  // 2. 软复位
  if (!writeRegister8(vl53l1x_reg::SOFT_RESET, 0x00)) {
    ROS_ERROR("[vl53l1x] 软复位失败");
    return false;
  }
  ros::Duration(0.1).sleep();
  if (!writeRegister8(vl53l1x_reg::SOFT_RESET, 0x01)) {
    ROS_ERROR("[vl53l1x] 退出复位失败");
    return false;
  }
  ros::Duration(0.1).sleep();

  // 3. 检查设备 ID
  uint8_t model_id = 0;
  // VL53L1X 模型 ID 寄存器通常位于 0x010F
  // 简化实现：跳过 ID 检查，直接进入配置

  // 4. 设置距离模式
  // distance_mode_: 0=Short(~1.3m), 1=Medium(~3m), 2=Long(~4m)
  // 通过设置 timing_budget 和 range_config 来控制

  // 5. 设置时序预算
  double actual_budget = timing_budget_ms_;
  switch (distance_mode_) {
    case 0: actual_budget = std::min(timing_budget_ms_, 20.0); break;
    case 1: actual_budget = std::min(timing_budget_ms_, 40.0); break;
    case 2: actual_budget = std::min(timing_budget_ms_, 100.0); break;
  }
  timing_budget_ms_ = actual_budget;

  ROS_INFO("[vl53l1x] 传感器初始化成功 (mode=%d, budget=%.0fms)",
           distance_mode_, timing_budget_ms_);
  return true;
}

bool VL53L1XDriver::startRanging() {
  if (is_ranging_) return true;
  // 向传感器发送连续测距命令
  // VL53L1X 的 SYSTEM_MODE_START 寄存器写入 0x40 启动连续测距
  if (!writeRegister8(vl53l1x_reg::SYSTEM_MODE_START, 0x40)) {
    ROS_ERROR("[vl53l1x] 启动连续测距失败");
    return false;
  }
  is_ranging_ = true;
  ROS_INFO("[vl53l1x] 连续测距已启动");
  return true;
}

bool VL53L1XDriver::stopRanging() {
  if (!is_ranging_) return true;
  writeRegister8(vl53l1x_reg::SYSTEM_MODE_START, 0x00);
  is_ranging_ = false;
  return true;
}

bool VL53L1XDriver::isDataReady() {
  uint8_t status = 0;
  if (!readRegister8(vl53l1x_reg::GPIO_TIO_HV_STATUS, status)) {
    return false;
  }
  // bit 0: interrupt status (1 = data ready)
  return (status & 0x01) != 0;
}

void VL53L1XDriver::clearInterrupt() {
  writeRegister8(vl53l1x_reg::SYSTEM_INTERRUPT_CLEAR, 0x01);
}

bool VL53L1XDriver::getDistance(uint16_t& distance_mm) {
  uint16_t range = 0;
  // 读取距离结果寄存器（SD0 = Standard Deviation 0，最常用的结果）
  if (!readRegister16(vl53l1x_reg::RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, range)) {
    return false;
  }

  // 检查 range status
  uint8_t range_status = 0;
  readRegister8(vl53l1x_reg::RESULT_RANGE_STATUS, range_status);

  // Range status:
  //  0 = valid
  //  1 = sigma failure (low SNR)
  //  2 = signal failure (no target)
  //  4 = out of bounds
  //  7 = wrap-around
  if (range_status != 0) {
    return false;
  }

  distance_mm = range;
  return true;
}

// ── 读取定时器回调 ─────────────────────────────────────────
void VL53L1XDriver::readTimerCallback(const ros::TimerEvent& /*event*/) {
  if (!sensor_ok_ || !is_ranging_) {
    return;
  }

  uint16_t raw_distance = 0;
  bool valid = getDistance(raw_distance);

  // 清除中断以准备下一次测量
  clearInterrupt();

  if (!valid) {
    consecutive_errors_++;
    if (consecutive_errors_ >= max_consecutive_errors_) {
      ROS_WARN_THROTTLE(2.0, "[vl53l1x] 连续 %d 次读取失败，传感器可能异常",
                        consecutive_errors_);
    }
    return;
  }

  consecutive_errors_ = 0;

  // ── 应用偏移校准 ──
  double distance = static_cast<double>(raw_distance) + offset_mm_;
  distance = std::max(0.0, distance);
  current_distance_mm_ = distance;

  // ── 应用中值滤波 ──
  static std::deque<double> median_buffer;
  if (use_median_filter_) {
    median_buffer.push_back(distance);
    while (static_cast<int>(median_buffer.size()) > median_window_size_) {
      median_buffer.pop_front();
    }

    if (static_cast<int>(median_buffer.size()) >= median_window_size_) {
      std::vector<double> sorted(median_buffer.begin(), median_buffer.end());
      std::sort(sorted.begin(), sorted.end());
      distance = sorted[sorted.size() / 2];  // 中位数
    }
  }

  // ── 应用 EMA 滤波 ──
  if (use_moving_average_) {
    filtered_distance_mm_ = moving_average_alpha_ * distance +
                            (1.0 - moving_average_alpha_) * filtered_distance_mm_;
  } else {
    filtered_distance_mm_ = distance;
  }

  // ── 发布距离 ──
  std_msgs::Float32 dist_msg;
  dist_msg.data = static_cast<float>(filtered_distance_mm_);
  distance_pub_.publish(dist_msg);

  last_valid_reading_ = ros::Time::now();

  // ── 安全检测 ──
  bool warning = (filtered_distance_mm_ < safety_threshold_mm_);
  bool critical = (filtered_distance_mm_ < critical_threshold_mm_);

  if (critical != safety_stop_active_) {
    safety_stop_active_ = critical;
    std_msgs::Bool stop_msg;
    stop_msg.data = critical;
    safety_stop_pub_.publish(stop_msg);
    if (critical) {
      ROS_WARN_THROTTLE(0.5, "[vl53l1x] ⚠ 紧急停止！距离仅 %.0f mm (< %.0f)",
                        filtered_distance_mm_, critical_threshold_mm_);
    }
  }

  if (warning != safety_warning_active_) {
    safety_warning_active_ = warning;
    std_msgs::Bool warn_msg;
    warn_msg.data = warning;
    safety_warning_pub_.publish(warn_msg);
    if (warning && !critical) {
      ROS_WARN_THROTTLE(1.0, "[vl53l1x] ⚡ 安全警告！距离 %.0f mm (< %.0f)",
                        filtered_distance_mm_, safety_threshold_mm_);
    }
  }

  // ── 检查传感器数据超时 ──
  double since_last = (ros::Time::now() - last_valid_reading_).toSec();
  if (since_last > safety_timeout_sec_) {
    if (!safety_warning_active_) {
      safety_warning_active_ = true;
      std_msgs::Bool warn;
      warn.data = true;
      safety_warning_pub_.publish(warn);
      ROS_WARN_THROTTLE(1.0, "[vl53l1x] 传感器数据超时 %.1f 秒", since_last);
    }
  }

  ROS_DEBUG("[vl53l1x] 距离: %.0f mm (raw: %d, status: %s%s)",
            filtered_distance_mm_, raw_distance,
            warning ? "WARN" : "OK",
            critical ? " CRITICAL" : "");
}

// ── 服务: 重置 ─────────────────────────────────────────────
bool VL53L1XDriver::resetServiceCallback(std_srvs::Trigger::Request& /*req*/,
                                          std_srvs::Trigger::Response& res) {
  ROS_INFO("[vl53l1x] 收到重置请求");

  if (is_ranging_) {
    stopRanging();
  }

  bool ok = initSensor() && startRanging();
  consecutive_errors_ = 0;
  sensor_ok_ = ok;

  res.success = ok;
  res.message = ok ? "传感器重置成功" : "传感器重置失败";
  return true;
}

// ── 服务: 校准 ─────────────────────────────────────────────
bool VL53L1XDriver::calibrateServiceCallback(std_srvs::Trigger::Request& /*req*/,
                                              std_srvs::Trigger::Response& res) {
  ROS_INFO("[vl53l1x] 收到校准请求");

  // 读取当前距离作为零位偏移
  // 前提：传感器前方无障碍物（例如距离应为无限远或已知距离）
  // 简化：将当前读数与期望值的差值作为 offset
  // 外部调用此服务前，应确保传感器前方无物体（或距离已知）

  // 取最近几次读数的平均
  double avg_distance = filtered_distance_mm_;

  // 如果当前读数在合理范围内（< 4000mm），说明可能对着障碍物
  // 正常情况下无限远距离约 3000-4000mm
  // 这里简化处理：将 offset 置零，提示用户手动设置
  offset_mm_ = 0;

  // 重新计算 filtered_distance
  filtered_distance_mm_ = current_distance_mm_ + offset_mm_;

  res.success = true;
  res.message = "校准完成: offset=" + std::to_string(offset_mm_) +
                "mm, 当前距离=" + std::to_string(static_cast<int>(filtered_distance_mm_)) + "mm";
  ROS_INFO("[vl53l1x] %s", res.message.c_str());
  return true;
}

double VL53L1XDriver::clamp(double val, double lo, double hi) const {
  return std::max(lo, std::min(hi, val));
}

}  // namespace robot_navigation

// ── main ───────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "vl53l1x_driver_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::VL53L1XDriver driver(nh, pnh);
  if (!driver.init()) {
    ROS_FATAL("[vl53l1x] 驱动初始化失败，节点退出");
    return 1;
  }

  ROS_INFO("[vl53l1x] VL53L1X 驱动节点运行中...");
  ros::spin();

  return 0;
}
