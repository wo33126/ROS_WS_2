/**
 * @file servo_controller_node.cpp
 * @brief 舵机控制节点 — 基于 pigpio 库驱动 SG90 舵机
 *
 * 使用 pigpio 的 hardware PWM 或 software PWM 控制舵机角度。
 * 依赖: libpigpio-dev, pigpiod 守护进程
 */

#include "arm_and_gripper/servo_controller.h"

#include <unistd.h>
#include <pigpio.h>

namespace arm_and_gripper {

// ── 构造 ───────────────────────────────────────────────────
ServoController::ServoController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , servo1_gpio_(18)
    , servo2_gpio_(19)
    , servo1_min_pw_us_(500)
    , servo1_max_pw_us_(2500)
    , servo2_min_pw_us_(500)
    , servo2_max_pw_us_(2500)
    , servo1_default_angle_(0.0)
    , servo2_default_angle_(0.0)
{}

ServoController::~ServoController() {
  // 回到默认角度（闭合/安全位置）
  setServoAngle(servo1_gpio_, servo1_default_angle_,
                servo1_min_pw_us_, servo1_max_pw_us_);
  setServoAngle(servo2_gpio_, servo2_default_angle_,
                servo2_min_pw_us_, servo2_max_pw_us_);
  gpioTerminate();
}

// ── 初始化 ─────────────────────────────────────────────────
bool ServoController::init() {
  // ── 参数读取 ──
  pnh_.param<int>("servo1_gpio", servo1_gpio_, 18);
  pnh_.param<int>("servo2_gpio", servo2_gpio_, 19);
  pnh_.param<int>("servo1_min_pw_us", servo1_min_pw_us_, 500);
  pnh_.param<int>("servo1_max_pw_us", servo1_max_pw_us_, 2500);
  pnh_.param<int>("servo2_min_pw_us", servo2_min_pw_us_, 500);
  pnh_.param<int>("servo2_max_pw_us", servo2_max_pw_us_, 2500);
  pnh_.param<double>("servo1_default_angle", servo1_default_angle_, 0.0);
  pnh_.param<double>("servo2_default_angle", servo2_default_angle_, 0.0);

  // ── 初始化 pigpio ──
  if (gpioInitialise() < 0) {
    ROS_FATAL("[servo_controller] pigpio 初始化失败！请确认 pigpiod 已运行: sudo pigpiod");
    return false;
  }
  ROS_INFO("[servo_controller] pigpio 初始化成功");

  // 设置 GPIO 为输出
  gpioSetMode(servo1_gpio_, PI_OUTPUT);
  gpioSetMode(servo2_gpio_, PI_OUTPUT);

  // ── 初始化舵机到默认角度 ──
  setServoAngle(servo1_gpio_, servo1_default_angle_,
                servo1_min_pw_us_, servo1_max_pw_us_);
  setServoAngle(servo2_gpio_, servo2_default_angle_,
                servo2_min_pw_us_, servo2_max_pw_us_);

  // ── 服务 ──
  servo_cmd_srv_ = nh_.advertiseService(
      "/servo_command", &ServoController::servoCommandCallback, this);

  ROS_INFO("[servo_controller] 初始化完成");
  ROS_INFO("[servo_controller]   舵机1: GPIO%d, 脉宽 %d-%d us",
           servo1_gpio_, servo1_min_pw_us_, servo1_max_pw_us_);
  ROS_INFO("[servo_controller]   舵机2: GPIO%d, 脉宽 %d-%d us",
           servo2_gpio_, servo2_min_pw_us_, servo2_max_pw_us_);
  ROS_INFO("[servo_controller]   /servo_command 服务已就绪");

  return true;
}

// ── 服务回调 ───────────────────────────────────────────────
bool ServoController::servoCommandCallback(
    arm_and_gripper::ServoCommand::Request& req,
    arm_and_gripper::ServoCommand::Response& res) {
  if (req.servo_id != 1 && req.servo_id != 2) {
    res.success = false;
    res.message = "无效的舵机编号: " + std::to_string(req.servo_id) + " (仅支持 1 或 2)";
    ROS_WARN("[servo_controller] %s", res.message.c_str());
    return true;
  }

  if (req.angle < 0.0 || req.angle > 180.0) {
    res.success = false;
    res.message = "角度超出范围 [0, 180]: " + std::to_string(req.angle);
    ROS_WARN("[servo_controller] %s", res.message.c_str());
    return true;
  }

  int gpio = (req.servo_id == 1) ? servo1_gpio_ : servo2_gpio_;
  int min_pw = (req.servo_id == 1) ? servo1_min_pw_us_ : servo2_min_pw_us_;
  int max_pw = (req.servo_id == 1) ? servo1_max_pw_us_ : servo2_max_pw_us_;

  if (setServoAngle(gpio, req.angle, min_pw, max_pw)) {
    res.success = true;
    res.message = "舵机" + std::to_string(req.servo_id)
                  + " 已设置为 " + std::to_string(req.angle) + "°";
    ROS_INFO("[servo_controller] %s", res.message.c_str());
  } else {
    res.success = false;
    res.message = "舵机" + std::to_string(req.servo_id) + " 设置失败";
    ROS_ERROR("[servo_controller] %s", res.message.c_str());
  }

  return true;
}

// ── 设置舵机角度 ───────────────────────────────────────────
bool ServoController::setServoAngle(int gpio_pin, double angle_deg,
                                    int min_pw_us, int max_pw_us) {
  // 角度 → 脉宽映射
  // SG90: 0°≈500μs, 90°≈1500μs, 180°≈2500μs（可配置）
  angle_deg = std::max(0.0, std::min(180.0, angle_deg));
  int pulse_width = min_pw_us +
      static_cast<int>((angle_deg / 180.0) * (max_pw_us - min_pw_us));

  int result = gpioServo(gpio_pin, pulse_width);
  if (result != 0) {
    ROS_ERROR("[servo_controller] gpioServo(GPIO%d, %dus) 失败: %d",
              gpio_pin, pulse_width, result);
    return false;
  }

  ROS_DEBUG("[servo_controller] GPIO%d → %.0f° → %d us", gpio_pin, angle_deg, pulse_width);
  return true;
}

}  // namespace arm_and_gripper

// ── 主函数 ─────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "servo_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  arm_and_gripper::ServoController controller(nh, pnh);

  if (!controller.init()) {
    ROS_FATAL("[servo_controller_node] 初始化失败，退出");
    return 1;
  }

  ros::spin();
  return 0;
}
