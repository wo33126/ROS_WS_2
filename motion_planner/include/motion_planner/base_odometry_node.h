#pragma once

#include <array>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_ros/transform_broadcaster.h>

namespace motion_planner {

class BaseOdometryNode {
 public:
  BaseOdometryNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void motorStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void publishTimerCallback(const ros::TimerEvent& event);
  void publishOdometry(const ros::Time& stamp);
  void computeBodyVelocityFromWheelRpm(const std::array<double, 4>& wheel_rpm,
                                       double& vx,
                                       double& vy,
                                       double& omega) const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber motor_state_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher pose2d_pub_;
  ros::Publisher body_vel_pub_;
  ros::Timer publish_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  double wheel_radius_;
  double wheel_base_;
  double odom_publish_hz_;
  bool publish_tf_;
  std::string odom_frame_;
  std::string base_frame_;

  bool has_feedback_;
  std::array<double, 4> last_wheel_rpm_;
  double x_;
  double y_;
  double yaw_;
  double body_vx_;
  double body_vy_;
  double body_omega_;
  ros::Time last_update_time_;
};

}  // namespace motion_planner
