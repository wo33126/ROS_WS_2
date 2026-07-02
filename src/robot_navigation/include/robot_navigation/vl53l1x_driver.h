#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

namespace robot_navigation {

/**
 * @brief VL53L1X 激光测距传感器驱动节点
 *
 * 通过 I2C 接口读取 VL53L1X 传感器数据，发布距离值 (mm)。
 * 同时提供距离安全检测：当距离低于阈值时发布 /distance_safety_warning 信号，
 * 可供 cmd_vel_mux 的 safety 通道使用，实现底盘移动中的自动避障。
 *
 * 话题:
 *   - /vl53l1x_distance       (Float32): 当前距离值 (mm)
 *   - /vl53l1x/status         (String):  传感器状态
 *   - /distance_safety_warning (Bool):   距离过近警告（低于 safety_threshold_mm）
 *   - /distance_safety_stop   (Bool):   紧急停止信号（低于 critical_threshold_mm）
 *
 * 服务:
 *   - /vl53l1x/reset          (Trigger): 重置传感器
 *   - /vl53l1x/calibrate      (Trigger): 偏移校准
 */
class VL53L1XDriver {
 public:
  VL53L1XDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~VL53L1XDriver();

  bool init();

 private:
  // ── I2C 硬件操作 ──
  bool openI2C();
  void closeI2C();
  bool readRegister16(uint16_t reg, uint16_t& value);
  bool writeRegister16(uint16_t reg, uint16_t value);
  bool readRegister8(uint16_t reg, uint8_t& value);
  bool writeRegister8(uint16_t reg, uint8_t value);

  // ── 传感器控制 ──
  bool initSensor();
  bool startRanging();
  bool stopRanging();
  bool getDistance(uint16_t& distance_mm);
  bool isDataReady();
  void clearInterrupt();

  // ── ROS 回调 ──
  void readTimerCallback(const ros::TimerEvent& event);
  bool resetServiceCallback(std_srvs::Trigger::Request& req,
                            std_srvs::Trigger::Response& res);
  bool calibrateServiceCallback(std_srvs::Trigger::Request& req,
                                std_srvs::Trigger::Response& res);

  // ── 工具 ──
  double clamp(double val, double lo, double hi) const;

  // ── ROS 接口 ──
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher distance_pub_;
  ros::Publisher status_pub_;
  ros::Publisher safety_warning_pub_;
  ros::Publisher safety_stop_pub_;
  ros::Timer   read_timer_;

  ros::ServiceServer reset_srv_;
  ros::ServiceServer calibrate_srv_;

  // ── I2C 参数 ──
  std::string i2c_device_;
  int         i2c_fd_;
  uint8_t     i2c_address_;       // 默认 0x29

  // ── 发布话题名称（可配置，支持多传感器） ──
  std::string topic_name_;        // 默认 "/vl53l1x_distance"

  // ── 传感器参数 ──
  double read_rate_hz_;           // 读取频率
  int    distance_mode_;          // 0=Short, 1=Medium, 2=Long
  double timing_budget_ms_;       // 测量时间预算

  // ── 滤波参数 ──
  bool   use_median_filter_;      // 中值滤波
  int    median_window_size_;     // 滤波窗口
  bool   use_moving_average_;     // 移动平均
  double moving_average_alpha_;   // EMA 系数

  // ── 安全距离参数 ──
  double safety_threshold_mm_;    // 安全警告阈值
  double critical_threshold_mm_;  // 紧急停止阈值
  double safety_timeout_sec_;     // 安全信号超时

  // ── 校准 ──
  int    offset_mm_;              // 距离偏移校准 (mm)

  // ── 状态 ──
  double current_distance_mm_;
  double filtered_distance_mm_;
  bool   sensor_ok_;
  bool   is_ranging_;
  int    consecutive_errors_;
  int    max_consecutive_errors_;

  // ── 安全状态 ──
  bool   safety_warning_active_;
  bool   safety_stop_active_;
  ros::Time last_valid_reading_;
};

}  // namespace robot_navigation
