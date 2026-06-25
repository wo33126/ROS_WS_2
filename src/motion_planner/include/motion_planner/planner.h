#pragma once

#include <geometry_msgs/Twist.h>
#include <motion_planner/kinematics.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

namespace motion_planner {

class PlannerNode {
 public:
  PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  double clamp(double val, double min_val, double max_val) const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher motor_cmd_pub_;

  Kinematics kinematics_;

  double max_linear_vel_;
  double max_angular_vel_;
  double max_rpm_;
  bool print_debug_log_;
};

}  // namespace motion_planner
