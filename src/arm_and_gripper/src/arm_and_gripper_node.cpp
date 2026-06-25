/**
 * @file arm_and_gripper_node.cpp
 * @brief 药品摆放主控制节点
 *
 * 协调机械臂电机（Y42步进/CAN2地址5）和两个SG90舵机，
 * 在微调完成后自动执行药品放置动作序列。
 *
 * 触发:
 *   - /fine_tuning_done 话题（自动）
 *   - /place_medicine   服务（手动）
 */

#include "arm_and_gripper/arm_and_gripper_controller.h"

#include <cerrno>
#include <cmath>
#include <cstring>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace arm_and_gripper {

// ── 构造 ───────────────────────────────────────────────────
ArmAndGripperController::ArmAndGripperController(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , can_device_("can1")
    , arm_can_id_(0x205)
    , position_cmd_code_(0x10)
    , arm_angle_scale_(1000.0 / 360.0)  // 默认: 360° = 1000 counts → 约 2.78 counts/°
    , socket_fd_(-1)
    , default_arm_angle_(90.0)
    , default_servo1_open_angle_(90.0)
    , default_servo2_open_angle_(90.0)
    , default_reset_arm_(true)
    , post_servo1_delay_s_(0.5)
    , post_servo2_delay_s_(1.0)
    , arm_move_timeout_s_(5.0)
    , push_distance_m_(0.05)
    , push_velocity_(0.03)
    , push_timeout_s_(5.0)
    , push_cmd_code_(0x11)
    , sequence_running_(false)
{}

ArmAndGripperController::~ArmAndGripperController() {
  closeCanSocket();
}

// ── 初始化 ─────────────────────────────────────────────────
bool ArmAndGripperController::init() {
  // ── 参数读取 ──
  pnh_.param<std::string>("can_device", can_device_, "can1");
  int can_id_int = static_cast<int>(arm_can_id_);
  pnh_.param("arm_can_id", can_id_int, can_id_int);
  arm_can_id_ = static_cast<uint32_t>(can_id_int);
  int cmd_code_int = static_cast<int>(position_cmd_code_);
  pnh_.param("position_cmd_code", cmd_code_int, cmd_code_int);
  position_cmd_code_ = static_cast<uint8_t>(cmd_code_int);
  pnh_.param<double>("arm_angle_scale", arm_angle_scale_, arm_angle_scale_);

  pnh_.param<double>("default_arm_angle", default_arm_angle_, 90.0);
  pnh_.param<double>("default_servo1_open_angle", default_servo1_open_angle_, 90.0);
  pnh_.param<double>("default_servo2_open_angle", default_servo2_open_angle_, 90.0);
  pnh_.param<bool>("default_reset_arm", default_reset_arm_, true);
  pnh_.param<double>("post_servo1_delay_s", post_servo1_delay_s_, 0.5);
  pnh_.param<double>("post_servo2_delay_s", post_servo2_delay_s_, 1.0);
  pnh_.param<double>("arm_move_timeout_s", arm_move_timeout_s_, 5.0);

  // ── 推出动作参数 ──
  pnh_.param<double>("push_distance_m", push_distance_m_, 0.05);
  pnh_.param<double>("push_velocity", push_velocity_, 0.03);
  pnh_.param<double>("push_timeout_s", push_timeout_s_, 5.0);
  int push_cmd_int = static_cast<int>(push_cmd_code_);
  pnh_.param("push_cmd_code", push_cmd_int, push_cmd_int);
  push_cmd_code_ = static_cast<int8_t>(push_cmd_int);

  // ── 订阅微调完成信号 ──
  fine_tuning_done_sub_ = nh_.subscribe(
      "/fine_tuning_done", 1,
      &ArmAndGripperController::fineTuningDoneCallback, this);

  // ── 服务: 手动触发药品摆放 ──
  place_medicine_srv_ = nh_.advertiseService(
      "/place_medicine", &ArmAndGripperController::placeMedicineCallback, this);

  // ── 服务: 机械臂放置药品（推出动作） ──
  arm_place_medicine_srv_ = nh_.advertiseService(
      "/arm_place_medicine", &ArmAndGripperController::armPlaceMedicineCallback, this);

  // ── 舵机控制客户端 ──
  // 等待 servo_controller_node 的服务
  ROS_INFO("[arm_and_gripper] 等待 /servo_command 服务...");
  if (!ros::service::waitForService("/servo_command", ros::Duration(5.0))) {
    ROS_WARN("[arm_and_gripper] /servo_command 服务未就绪，舵机控制将不可用");
  }
  servo_cmd_client_ = nh_.serviceClient<arm_and_gripper::ServoCommand>("/servo_command");

  // ── 初始化 CAN ──
  if (!initCanSocket()) {
    ROS_WARN("[arm_and_gripper] CAN 初始化失败，将在首次使用时重试");
  }

  ROS_INFO("[arm_and_gripper] 初始化完成");
  ROS_INFO("[arm_and_gripper]   CAN设备: %s, CAN ID: 0x%03X", can_device_.c_str(), arm_can_id_);
  ROS_INFO("[arm_and_gripper]   默认机械臂角度: %.0f°", default_arm_angle_);
  ROS_INFO("[arm_and_gripper]   舵机1张开角度: %.0f°, 舵机2张开角度: %.0f°",
           default_servo1_open_angle_, default_servo2_open_angle_);
  ROS_INFO("[arm_and_gripper]   订阅 /fine_tuning_done, 服务 /place_medicine 已就绪");

  return true;
}

// ── /fine_tuning_done 回调 ─────────────────────────────────
void ArmAndGripperController::fineTuningDoneCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!msg->data) return;  // 仅响应 true

  ROS_INFO("[arm_and_gripper] ====== 收到微调完成信号，开始药品摆放 ======");
  executePlaceSequence(default_arm_angle_,
                       default_servo1_open_angle_,
                       default_servo2_open_angle_,
                       default_reset_arm_);
}

// ── /place_medicine 服务回调 ───────────────────────────────
bool ArmAndGripperController::placeMedicineCallback(
    arm_and_gripper::PlaceMedicine::Request& req,
    arm_and_gripper::PlaceMedicine::Response& res) {
  if (sequence_running_.load()) {
    res.success = false;
    res.message = "已有药品摆放动作正在执行中，请稍后重试";
    ROS_WARN("[arm_and_gripper] %s", res.message.c_str());
    return true;
  }

  double arm_angle = (req.arm_angle >= 0.0) ? req.arm_angle : default_arm_angle_;
  double s1_angle = (req.servo1_open_angle >= 0.0) ? req.servo1_open_angle : default_servo1_open_angle_;
  double s2_angle = (req.servo2_open_angle >= 0.0) ? req.servo2_open_angle : default_servo2_open_angle_;

  ROS_INFO("[arm_and_gripper] ====== 收到手动药品摆放请求 ======");

  bool ok = executePlaceSequence(arm_angle, s1_angle, s2_angle, req.reset_arm);
  res.success = ok;
  res.message = ok ? "药品摆放动作序列完成" : "药品摆放动作序列失败，请查看日志";
  return true;
}

// ── 执行动作序列 ───────────────────────────────────────────
bool ArmAndGripperController::executePlaceSequence(
    double arm_angle,
    double servo1_angle,
    double servo2_angle,
    bool reset_arm) {
  std::lock_guard<std::mutex> lock(seq_mutex_);
  sequence_running_.store(true);

  bool all_ok = true;

  // ── 步骤1: 机械臂旋转到指定角度 ──
  ROS_INFO("[arm_and_gripper] [1/5] 机械臂旋转到 %.0f° ...", arm_angle);
  if (!sendArmAngleCommand(arm_angle)) {
    ROS_ERROR("[arm_and_gripper] 机械臂角度指令发送失败！");
    all_ok = false;
  } else {
    // 等待机械臂到位（简单延时，可通过编码器反馈改进）
    ROS_INFO("[arm_and_gripper] 等待机械臂到位 (%.1f s)...", arm_move_timeout_s_);
    ros::Duration(arm_move_timeout_s_).sleep();
    ROS_INFO("[arm_and_gripper] 机械臂已就位");
  }

  // ── 步骤2: 舵机1张开 ──
  ROS_INFO("[arm_and_gripper] [2/5] 舵机1张开到 %.0f° ...", servo1_angle);
  if (!callServoAngle(1, servo1_angle)) {
    ROS_ERROR("[arm_and_gripper] 舵机1控制失败！");
    all_ok = false;
  }
  ros::Duration(post_servo1_delay_s_).sleep();

  // ── 步骤3: 舵机2张开 ──
  ROS_INFO("[arm_and_gripper] [3/5] 舵机2张开到 %.0f° ...", servo2_angle);
  if (!callServoAngle(2, servo2_angle)) {
    ROS_ERROR("[arm_and_gripper] 舵机2控制失败！");
    all_ok = false;
  }
  ros::Duration(post_servo2_delay_s_).sleep();

  // ── 步骤4: 舵机1闭合 ──
  ROS_INFO("[arm_and_gripper] [4/5] 舵机1闭合 (回到0°) ...");
  callServoAngle(1, 0.0);

  // ── 步骤5: 机械臂复位（可选） ──
  if (reset_arm) {
    ROS_INFO("[arm_and_gripper] [5/5] 机械臂复位到 0° ...");
    if (!sendArmAngleCommand(0.0)) {
      ROS_ERROR("[arm_and_gripper] 机械臂复位失败！");
      all_ok = false;
    } else {
      ros::Duration(arm_move_timeout_s_).sleep();
    }
  } else {
    ROS_INFO("[arm_and_gripper] [5/5] 跳过机械臂复位");
  }

  sequence_running_.store(false);

  if (all_ok) {
    ROS_INFO("[arm_and_gripper] ====== 药品摆放动作序列完成 ======");
  } else {
    ROS_ERROR("[arm_and_gripper] ====== 药品摆放动作序列有错误 ======");
  }

  return all_ok;
}

// ── CAN Socket 初始化 ──────────────────────────────────────
bool ArmAndGripperController::initCanSocket() {
  if (socket_fd_ >= 0) return true;

  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    ROS_ERROR("[arm_and_gripper] CAN socket 创建失败: %s", std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_device_.c_str());
  if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ROS_ERROR("[arm_and_gripper] ioctl(SIOCGIFINDEX) 失败 for %s: %s",
              can_device_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR("[arm_and_gripper] CAN bind 失败 on %s: %s",
              can_device_.c_str(), std::strerror(errno));
    close(fd);
    return false;
  }

  socket_fd_ = fd;
  ROS_INFO("[arm_and_gripper] CAN Socket 已连接: %s", can_device_.c_str());
  return true;
}

void ArmAndGripperController::closeCanSocket() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

// ── 发送机械臂角度指令 (CAN) ───────────────────────────────
bool ArmAndGripperController::sendArmAngleCommand(double angle_deg) {
  if (socket_fd_ < 0 && !initCanSocket()) {
    ROS_ERROR("[arm_and_gripper] CAN socket 不可用，无法发送角度指令");
    return false;
  }

  // Y42 原生协议：使用地址 5 对应的双帧指令。
  // arm_can_id_ 仍保留配置入口，但这里只取低 8 位作为电机地址。
  const uint32_t motor_address = arm_can_id_ & 0xFFu;
  if (motor_address == 0) {
    ROS_ERROR("[arm_and_gripper] 无效的电机地址: 0x%03X", arm_can_id_);
    return false;
  }

  const uint32_t frame0_id = (motor_address << 8) | 0x00u;
  const uint32_t frame1_id = (motor_address << 8) | 0x01u;

  // 协议中的速度与角度单位
  const uint16_t speed_rpm_x10 = 500;  // 50 RPM
  const uint32_t target_degrees_x10 = static_cast<uint32_t>(std::round(std::fabs(angle_deg) * 10.0));
  const uint8_t direction = (angle_deg >= 0.0) ? 1u : 0u;

  auto send_frame = [this](const struct can_frame& frame) -> bool {
    int nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != static_cast<int>(sizeof(frame))) {
      ROS_ERROR("[arm_and_gripper] CAN 写入失败: %s", std::strerror(errno));
      return false;
    }
    return true;
  };

  struct can_frame frame0;
  std::memset(&frame0, 0, sizeof(frame0));
  frame0.can_id = frame0_id | CAN_EFF_FLAG;
  frame0.can_dlc = 8;
  frame0.data[0] = 0xFB;
  frame0.data[1] = direction;
  frame0.data[2] = static_cast<uint8_t>((speed_rpm_x10 >> 8) & 0xFF);
  frame0.data[3] = static_cast<uint8_t>(speed_rpm_x10 & 0xFF);
  frame0.data[4] = static_cast<uint8_t>((target_degrees_x10 >> 24) & 0xFF);
  frame0.data[5] = static_cast<uint8_t>((target_degrees_x10 >> 16) & 0xFF);
  frame0.data[6] = static_cast<uint8_t>((target_degrees_x10 >> 8) & 0xFF);
  frame0.data[7] = static_cast<uint8_t>(target_degrees_x10 & 0xFF);

  struct can_frame frame1;
  std::memset(&frame1, 0, sizeof(frame1));
  frame1.can_id = frame1_id | CAN_EFF_FLAG;
  frame1.can_dlc = 4;
  frame1.data[0] = 0xFB;
  frame1.data[1] = 0x02;  // mode = 2，从当前位置打断
  frame1.data[2] = 0x00; // 保留
  frame1.data[3] = 0x6B;  // 校验

  if (!send_frame(frame0) || !send_frame(frame1)) {
    return false;
  }

  ROS_INFO("[arm_and_gripper] CAN TX: ID=0x%03X/0x%03X, dir=%u, speed=50RPM, angle=%.1f°",
           frame0_id, frame1_id, direction, angle_deg);
  return true;
}

// ── 调用舵机服务 ───────────────────────────────────────────
bool ArmAndGripperController::callServoAngle(uint8_t servo_id, double angle_deg) {
  if (!servo_cmd_client_.exists()) {
    ROS_WARN("[arm_and_gripper] /servo_command 服务不可用，等待...");
    if (!servo_cmd_client_.waitForExistence(ros::Duration(3.0))) {
      ROS_ERROR("[arm_and_gripper] /servo_command 服务超时不可用");
      return false;
    }
  }

  arm_and_gripper::ServoCommand srv;
  srv.request.servo_id = servo_id;
  srv.request.angle = static_cast<float>(angle_deg);

  if (servo_cmd_client_.call(srv)) {
    if (srv.response.success) {
      ROS_INFO("[arm_and_gripper] %s", srv.response.message.c_str());
      return true;
    } else {
      ROS_ERROR("[arm_and_gripper] 舵机服务返回失败: %s", srv.response.message.c_str());
      return false;
    }
  } else {
    ROS_ERROR("[arm_and_gripper] 调用舵机服务失败");
    return false;
  }
}

// ── /arm_place_medicine 服务回调 ───────────────────────────
bool ArmAndGripperController::armPlaceMedicineCallback(
    arm_and_gripper::ArmPlaceMedicine::Request& req,
    arm_and_gripper::ArmPlaceMedicine::Response& res) {
  if (sequence_running_.load()) {
    res.success = false;
    res.message = "已有动作正在执行中";
    ROS_WARN("[arm_and_gripper] %s", res.message.c_str());
    return true;
  }

  if (req.bed_id != 1 && req.bed_id != 3) {
    res.success = false;
    res.message = "无效的病床编号: " + std::to_string(req.bed_id);
    return true;
  }

  ROS_INFO("[arm_and_gripper] ====== 收到药品放置请求: bed_id=%d ======", req.bed_id);

  std::lock_guard<std::mutex> lock(seq_mutex_);
  sequence_running_.store(true);

  bool ok = true;

  // ── 步骤1: 推出（将药品从药箱推到床头柜圆圈内） ──
  ROS_INFO("[arm_and_gripper] [1/2] 推出 %.2f m ...", push_distance_m_);
  if (!sendArmLinearCommand(push_distance_m_)) {
    ROS_ERROR("[arm_and_gripper] 推出动作失败！");
    ok = false;
  } else {
    ros::Duration(push_timeout_s_).sleep();
  }

  // ── 步骤2: 退回 ──
  ROS_INFO("[arm_and_gripper] [2/2] 退回 ...");
  if (!sendArmLinearCommand(-push_distance_m_)) {
    ROS_ERROR("[arm_and_gripper] 退回动作失败！");
    ok = false;
  } else {
    ros::Duration(push_timeout_s_).sleep();
  }

  sequence_running_.store(false);

  res.success = ok;
  res.message = ok ? "药品放置完成" : "药品放置失败，请查看日志";

  if (ok) {
    ROS_INFO("[arm_and_gripper] ====== 药品放置完成 ======");
  } else {
    ROS_ERROR("[arm_and_gripper] ====== 药品放置失败 ======");
  }

  return true;
}

// ── 发送线性移动指令（推出/退回） ──────────────────────────
bool ArmAndGripperController::sendArmLinearCommand(double distance_m) {
  if (socket_fd_ < 0 && !initCanSocket()) {
    ROS_ERROR("[arm_and_gripper] CAN socket 不可用，无法发送线性指令");
    return false;
  }

  // 将距离转换为编码器计数值（假设相同比例因子）
  int32_t target_counts = static_cast<int32_t>(std::round(distance_m * arm_angle_scale_ * 100.0));

  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = arm_can_id_;
  frame.can_dlc = 8;

  // 数据格式:
  // [0]: 指令码 (push_cmd_code_, 线性移动模式)
  // [1]: 电机索引 (0)
  // [2-5]: 目标位移 (int32, little-endian)
  // [6-7]: 保留
  frame.data[0] = static_cast<uint8_t>(push_cmd_code_);
  frame.data[1] = 0;

  frame.data[2] = static_cast<uint8_t>(target_counts & 0xFF);
  frame.data[3] = static_cast<uint8_t>((target_counts >> 8) & 0xFF);
  frame.data[4] = static_cast<uint8_t>((target_counts >> 16) & 0xFF);
  frame.data[5] = static_cast<uint8_t>((target_counts >> 24) & 0xFF);
  frame.data[6] = 0;
  frame.data[7] = 0;

  int nbytes = write(socket_fd_, &frame, sizeof(frame));
  if (nbytes != static_cast<int>(sizeof(frame))) {
    ROS_ERROR("[arm_and_gripper] CAN 写入失败: %s", std::strerror(errno));
    return false;
  }

  ROS_INFO("[arm_and_gripper] CAN TX (Linear): ID=0x%03X, dist=%.3f m → counts=%d",
           arm_can_id_, distance_m, target_counts);
  return true;
}

}  // namespace arm_and_gripper

// ── 主函数 ─────────────────────────────────────────────────
int main(int argc, char** argv) {
  ros::init(argc, argv, "arm_and_gripper_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  arm_and_gripper::ArmAndGripperController controller(nh, pnh);

  if (!controller.init()) {
    ROS_FATAL("[arm_and_gripper_node] 初始化失败，退出");
    return 1;
  }

  ros::spin();
  return 0;
}
