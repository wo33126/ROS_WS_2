/**
 * @file open_medicine_box.cpp
 * @brief 药箱开启控制节点
 *
 * 提供 /open_medicine_box 服务，根据 box_id 通过 CAN1 发送命令给 STM32，
 * 控制对应药箱的舵机旋转 90° 打开箱盖。
 *
 * CAN协议（自定义）：
 *   CAN ID: 0x206
 *   Data[0]: 指令码 (0x20 = 舵机控制)
 *   Data[1]: 舵机编号 (box_id: 1 或 3)
 *   Data[2]: 目标角度 (0~180°, 90° 为打开)
 *   Data[3-7]: 保留
 */

#include <ros/ros.h>
#include <robot_navigation/OpenMedicineBox.h>

#include <cerrno>
#include <cstring>
#include <string>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_navigation {

class OpenMedicineBoxNode {
 public:
  OpenMedicineBoxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
      , socket_fd_(-1)
      , can_device_("can1")
      , can_id_(0x206)
      , servo_cmd_code_(0x20)
      , open_angle_deg_(90.0)
      , hold_duration_sec_(1.0)
  {
    pnh_.param<std::string>("can_device", can_device_, "can1");
    int can_id_int = static_cast<int>(can_id_);
    pnh_.param("can_id", can_id_int, can_id_int);
    can_id_ = static_cast<uint32_t>(can_id_int);
    int cmd_int = static_cast<int>(servo_cmd_code_);
    pnh_.param("servo_cmd_code", cmd_int, cmd_int);
    servo_cmd_code_ = static_cast<uint8_t>(cmd_int);
    pnh_.param<double>("open_angle_deg", open_angle_deg_, 90.0);
    pnh_.param<double>("hold_duration_sec", hold_duration_sec_, 1.0);
  }

  ~OpenMedicineBoxNode() {
    if (socket_fd_ >= 0) close(socket_fd_);
  }

  bool init() {
    // ── 服务 ──
    service_ = nh_.advertiseService("/open_medicine_box",
                                    &OpenMedicineBoxNode::callback, this);

    // ── 初始化 CAN ──
    if (!initCan()) {
      ROS_WARN("[open_medicine_box] CAN 初始化失败，将在首次调用时重试");
    }

    ROS_INFO("[open_medicine_box] 初始化完成");
    ROS_INFO("[open_medicine_box]   CAN: %s, CAN ID: 0x%03X", can_device_.c_str(), can_id_);
    ROS_INFO("[open_medicine_box]   打开角度: %.0f°, 保持时间: %.1f s",
             open_angle_deg_, hold_duration_sec_);
    ROS_INFO("[open_medicine_box]   服务 /open_medicine_box 已就绪");

    return true;
  }

 private:
  bool callback(robot_navigation::OpenMedicineBox::Request& req,
                robot_navigation::OpenMedicineBox::Response& res) {
    if (req.box_id != 1 && req.box_id != 3) {
      res.success = false;
      res.message = "无效的药箱编号: " + std::to_string(req.box_id) + " (仅支持 1 或 3)";
      ROS_ERROR("[open_medicine_box] %s", res.message.c_str());
      return true;
    }

    ROS_INFO("[open_medicine_box] 收到开箱请求: box_id=%d", req.box_id);

    if (!sendServoCommand(static_cast<uint8_t>(req.box_id), open_angle_deg_)) {
      res.success = false;
      res.message = "CAN 发送失败，无法控制药箱 " + std::to_string(req.box_id);
      ROS_ERROR("[open_medicine_box] %s", res.message.c_str());
      return true;
    }

    // 等待舵机执行到位
    ros::Duration(hold_duration_sec_).sleep();

    res.success = true;
    res.message = "药箱 " + std::to_string(req.box_id) + " 已打开";
    ROS_INFO("[open_medicine_box] %s", res.message.c_str());
    return true;
  }

  bool initCan() {
    if (socket_fd_ >= 0) return true;

    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
      ROS_ERROR("[open_medicine_box] CAN socket 创建失败: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_device_.c_str());
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      ROS_ERROR("[open_medicine_box] ioctl(SIOCGIFINDEX) 失败: %s", std::strerror(errno));
      close(fd);
      return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
      ROS_ERROR("[open_medicine_box] bind 失败: %s", std::strerror(errno));
      close(fd);
      return false;
    }

    socket_fd_ = fd;
    ROS_INFO("[open_medicine_box] CAN Socket 已连接: %s", can_device_.c_str());
    return true;
  }

  bool sendServoCommand(uint8_t box_id, double angle_deg) {
    if (socket_fd_ < 0 && !initCan()) {
      ROS_ERROR("[open_medicine_box] CAN socket 不可用");
      return false;
    }

    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = can_id_;
    frame.can_dlc = 8;

    frame.data[0] = servo_cmd_code_;     // 指令码
    frame.data[1] = box_id;              // 舵机编号
    frame.data[2] = static_cast<uint8_t>(
        std::min(180.0, std::max(0.0, angle_deg)));  // 目标角度
    frame.data[3] = 0;
    frame.data[4] = 0;
    frame.data[5] = 0;
    frame.data[6] = 0;
    frame.data[7] = 0;

    int nbytes = write(socket_fd_, &frame, sizeof(frame));
    if (nbytes != static_cast<int>(sizeof(frame))) {
      ROS_ERROR("[open_medicine_box] CAN 写入失败: %s", std::strerror(errno));
      return false;
    }

    ROS_INFO("[open_medicine_box] CAN TX: ID=0x%03X, box=%d, angle=%.0f°",
             can_id_, box_id, angle_deg);
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceServer service_;

  int socket_fd_;
  std::string can_device_;
  uint32_t can_id_;
  uint8_t servo_cmd_code_;
  double open_angle_deg_;
  double hold_duration_sec_;
};

}  // namespace robot_navigation

int main(int argc, char** argv) {
  ros::init(argc, argv, "open_medicine_box_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  robot_navigation::OpenMedicineBoxNode node(nh, pnh);
  if (!node.init()) {
    ROS_FATAL("[open_medicine_box] 初始化失败");
    return 1;
  }

  ros::spin();
  return 0;
}
