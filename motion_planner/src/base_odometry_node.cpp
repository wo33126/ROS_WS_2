#include "motion_planner/base_odometry_node.h"

#include <cmath>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace motion_planner {

namespace {

constexpr double kTwoPi = 2.0 * M_PI;

}  // namespace

BaseOdometryNode::BaseOdometryNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      wheel_radius_(0.05),
      wheel_base_(0.18),
      odom_publish_hz_(50.0),
      publish_tf_(true),
      odom_frame_("odom"),
      base_frame_("base_link"),
      has_feedback_(false),
      last_wheel_rpm_{0.0, 0.0, 0.0, 0.0},
      x_(0.0),
      y_(0.0),
      yaw_(0.0),
      body_vx_(0.0),
      body_vy_(0.0),
      body_omega_(0.0) {
  nh_.param("/robot/wheel_radius", wheel_radius_, wheel_radius_);
  nh_.param("/robot/wheel_base", wheel_base_, wheel_base_);
  pnh_.param("odom_publish_hz", odom_publish_hz_, odom_publish_hz_);
  pnh_.param("publish_tf", publish_tf_, publish_tf_);
  pnh_.param("odom_frame", odom_frame_, odom_frame_);
  pnh_.param("base_frame", base_frame_, base_frame_);

  motor_state_sub_ = nh_.subscribe("/motor_state", 20, &BaseOdometryNode::motorStateCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 20);
  pose2d_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/base_pose2d", 20);
  body_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_velocity", 20);

  const double period = 1.0 / std::max(1.0, odom_publish_hz_);
  publish_timer_ = nh_.createTimer(ros::Duration(period), &BaseOdometryNode::publishTimerCallback, this);
  last_update_time_ = ros::Time::now();
}

void BaseOdometryNode::motorStateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  if (msg->data.size() < 4) {
    ROS_WARN_THROTTLE(1.0, "Expected 4 motor feedback values for odometry, got %zu.", msg->data.size());
    return;
  }

  std::array<double, 4> wheel_rpm = {
      static_cast<double>(msg->data[0]),
      static_cast<double>(msg->data[1]),
      static_cast<double>(msg->data[2]),
      static_cast<double>(msg->data[3]),
  };

  const ros::Time now = ros::Time::now();
  if (!has_feedback_) {
    has_feedback_ = true;
    last_wheel_rpm_ = wheel_rpm;
    last_update_time_ = now;
    computeBodyVelocityFromWheelRpm(wheel_rpm, body_vx_, body_vy_, body_omega_);
    return;
  }

  const double dt = std::max(1e-3, (now - last_update_time_).toSec());
  computeBodyVelocityFromWheelRpm(wheel_rpm, body_vx_, body_vy_, body_omega_);

  const double cos_yaw = std::cos(yaw_);
  const double sin_yaw = std::sin(yaw_);
  const double world_vx = cos_yaw * body_vx_ - sin_yaw * body_vy_;
  const double world_vy = sin_yaw * body_vx_ + cos_yaw * body_vy_;

  x_ += world_vx * dt;
  y_ += world_vy * dt;
  yaw_ += body_omega_ * dt;

  last_wheel_rpm_ = wheel_rpm;
  last_update_time_ = now;
}

void BaseOdometryNode::publishTimerCallback(const ros::TimerEvent&) {
  if (!has_feedback_) {
    return;
  }
  publishOdometry(ros::Time::now());
}

void BaseOdometryNode::publishOdometry(const ros::Time& stamp) {
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x = body_vx_;
  odom.twist.twist.linear.y = body_vy_;
  odom.twist.twist.angular.z = body_omega_;
  odom_pub_.publish(odom);

  geometry_msgs::Pose2D pose2d;
  pose2d.x = x_;
  pose2d.y = y_;
  pose2d.theta = yaw_;
  pose2d_pub_.publish(pose2d);

  geometry_msgs::Twist twist;
  twist.linear.x = body_vx_;
  twist.linear.y = body_vy_;
  twist.angular.z = body_omega_;
  body_vel_pub_.publish(twist);

  if (publish_tf_) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(transform);
  }
}

void BaseOdometryNode::computeBodyVelocityFromWheelRpm(const std::array<double, 4>& wheel_rpm,
                                                       double& vx,
                                                       double& vy,
                                                       double& omega) const {
  std::array<double, 4> wheel_linear_vel = {0.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < wheel_rpm.size(); ++i) {
    wheel_linear_vel[i] = wheel_rpm[i] * kTwoPi * wheel_radius_ / 60.0;
  }

  vx = 0.5 * (wheel_linear_vel[0] - wheel_linear_vel[2]);
  vy = 0.5 * (wheel_linear_vel[1] - wheel_linear_vel[3]);
  omega = (wheel_linear_vel[0] + wheel_linear_vel[1] + wheel_linear_vel[2] + wheel_linear_vel[3]) /
          (4.0 * std::max(1e-6, wheel_base_));
}

}  // namespace motion_planner
