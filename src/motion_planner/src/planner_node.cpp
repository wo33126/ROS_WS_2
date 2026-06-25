#include "motion_planner/planner.h"

#include <algorithm>
#include <cmath>

namespace motion_planner {

PlannerNode::PlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      max_linear_vel_(1.0),
      max_angular_vel_(2.0),
      max_rpm_(3000.0),
      print_debug_log_(false) {
  kinematics_.loadFromRos(nh_, pnh_);

  nh_.param("/robot/max_linear_vel", max_linear_vel_, max_linear_vel_);
  nh_.param("/robot/max_angular_vel", max_angular_vel_, max_angular_vel_);
  nh_.param("/robot/max_rpm", max_rpm_, max_rpm_);

  pnh_.param("max_linear_vel", max_linear_vel_, max_linear_vel_);
  pnh_.param("max_angular_vel", max_angular_vel_, max_angular_vel_);
  pnh_.param("max_rpm", max_rpm_, max_rpm_);
  pnh_.param("print_debug_log", print_debug_log_, print_debug_log_);

  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 20, &PlannerNode::cmdVelCallback, this);
  motor_cmd_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/motor_velocity_cmd", 20);
}

double PlannerNode::clamp(double val, double min_val, double max_val) const {
  return std::max(min_val, std::min(max_val, val));
}

void PlannerNode::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  const double vx = clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
  const double vy = clamp(msg->linear.y, -max_linear_vel_, max_linear_vel_);
  const double omega = clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

  double wheel_linear_vel_mps[4] = {0.0, 0.0, 0.0, 0.0};
  kinematics_.twistToWheelLinearVelocities(vx, vy, omega, wheel_linear_vel_mps);

  std_msgs::Float32MultiArray out;
  out.data.resize(4, 0.0f);
  for (size_t i = 0; i < 4; ++i) {
    double rpm = kinematics_.linearToRpm(wheel_linear_vel_mps[i]);
    rpm = clamp(rpm, -max_rpm_, max_rpm_);
    out.data[i] = static_cast<float>(rpm);
  }

  if (print_debug_log_) {
    ROS_INFO_THROTTLE(0.5,
                      "cmd_vel(vx=%.3f, vy=%.3f, wz=%.3f) -> rpm[F,R,B,L]=[%.1f, %.1f, %.1f, %.1f]",
                      vx,
                      vy,
                      omega,
                      out.data[0],
                      out.data[1],
                      out.data[2],
                      out.data[3]);
  }

  motor_cmd_pub_.publish(out);
}

}  // namespace motion_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  motion_planner::PlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
