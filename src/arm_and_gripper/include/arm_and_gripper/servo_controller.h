#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <map>

#include <ros/ros.h>
#include <arm_and_gripper/ServoCommand.h>

namespace arm_and_gripper {

/**
 * @brief 舵机控制器 — 基于 pigpio 库的 PWM 舵机驱动
 *
 * 通过树莓派 GPIO 直接输出 PWM 控制 SG90 等小型舵机。
 * 提供 /servo_command ROS 服务，接受角度指令（0~180°）。
 *
 * 前置条件：
 *   - pigpio 守护进程已运行: sudo pigpiod
 *   - 或通过 launch 文件自动启动
 */
class ServoController {
 public:
  /**
   * @param nh  全局 NodeHandle
   * @param pnh 私有 NodeHandle（命名空间如 ~/servo_controller）
   */
  ServoController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ServoController();

  bool init();

 private:
  /** 服务回调：处理角度指令 */
  bool servoCommandCallback(
      arm_and_gripper::ServoCommand::Request& req,
      arm_and_gripper::ServoCommand::Response& res);

  /**
   * @brief 设置指定舵机的角度
   * @param gpio_pin   GPIO 引脚号 (BCM)
   * @param angle_deg  角度 (0~180)
   * @param min_pw_us  0° 对应脉宽 (μs)
   * @param max_pw_us  180° 对应脉宽 (μs)
   * @return 是否成功
   */
  bool setServoAngle(int gpio_pin, double angle_deg,
                     int min_pw_us, int max_pw_us);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::ServiceServer servo_cmd_srv_;

  // ── 参数 ──
  int servo1_gpio_;       // 舵机1 GPIO (BCM编号)
  int servo2_gpio_;       // 舵机2 GPIO (BCM编号)
  int servo1_min_pw_us_;  // 0° 脉宽
  int servo1_max_pw_us_;  // 180° 脉宽
  int servo2_min_pw_us_;
  int servo2_max_pw_us_;
  double servo1_default_angle_;
  double servo2_default_angle_;
};

}  // namespace arm_and_gripper
