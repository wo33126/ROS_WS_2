/**
 * @file open_medicine_box.cpp
 * @brief 药箱开启控制节点（使用公共 CAN 通信库）
 */

#include <ros/ros.h>
#include <robot_navigation/OpenMedicineBox.h>
#include "robot_navigation/can_utils.h"

#include <algorithm>
#include <cstring>
#include <string>

namespace robot_navigation {

class OpenMedicineBoxNode {
 public:
  OpenMedicineBoxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh)
      , pnh_(pnh)
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

  bool init() {
    service_ = nh_.advertiseService("/open_medicine_box",
                                    &OpenMedicineBoxNode::callback, this);

    if (!can_.open(can_device_)) {
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

    ros::Duration(hold_duration_sec_).sleep();

    res.success = true;
    res.message = "药箱 " + std::to_string(req.box_id) + " 已打开";
    ROS_INFO("[open_medicine_box] %s", res.message.c_str());
    return true;
  }

  bool sendServoCommand(uint8_t box_id, double angle_deg) {
    if (!can_.isOpen() && !can_.open(can_device_)) {
      ROS_ERROR("[open_medicine_box] CAN socket 不可用");
      return false;
    }

    uint8_t data[8] = {0};
    data[0] = servo_cmd_code_;
    data[1] = box_id;
    data[2] = static_cast<uint8_t>(std::min(180.0, std::max(0.0, angle_deg)));

    if (!can_.sendFrame(can_id_, data, 8)) {
      ROS_ERROR("[open_medicine_box] CAN 发送失败");
      return false;
    }

    ROS_INFO("[open_medicine_box] CAN TX: ID=0x%03X, box=%d, angle=%.0f°",
             can_id_, box_id, angle_deg);
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::ServiceServer service_;

  CanInterface can_;
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
