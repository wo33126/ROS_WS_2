#include "motion_planner/cmd_vel_mux_node.h"

#include <algorithm>
#include <cmath>

namespace motion_planner {

CmdVelMuxNode::CmdVelMuxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      publish_rate_hz_(30.0),
      cmd_timeout_sec_(0.5),
      max_linear_vel_(1.0),
      max_angular_vel_(2.0),
      max_linear_accel_(1.0),
      max_angular_accel_(2.0),
      estop_active_(false) {
  nh_.param("/robot/max_linear_vel", max_linear_vel_, max_linear_vel_);
  nh_.param("/robot/max_angular_vel", max_angular_vel_, max_angular_vel_);
  nh_.param("/robot/max_linear_accel", max_linear_accel_, max_linear_accel_);
  nh_.param("/robot/max_angular_accel", max_angular_accel_, max_angular_accel_);
  pnh_.param("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);
  pnh_.param("cmd_timeout_sec", cmd_timeout_sec_, cmd_timeout_sec_);

  fixed_route_sub_ = nh_.subscribe("/cmd_vel_fixed_route", 10, &CmdVelMuxNode::fixedRouteCallback, this);
  teleop_sub_ = nh_.subscribe("/cmd_vel_teleop", 10, &CmdVelMuxNode::teleopCallback, this);
  external_sub_ = nh_.subscribe("/cmd_vel_external", 10, &CmdVelMuxNode::externalCallback, this);
  safety_sub_ = nh_.subscribe("/cmd_vel_safety", 10, &CmdVelMuxNode::safetyCallback, this);
  estop_sub_ = nh_.subscribe("/emergency_stop", 10, &CmdVelMuxNode::estopCallback, this);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  selected_source_pub_ = nh_.advertise<std_msgs::String>("/cmd_vel_mux/selected_source", 10, true);
  estop_state_pub_ = nh_.advertise<std_msgs::Bool>("/cmd_vel_mux/estop_active", 10, true);

  sources_["fixed_route"] = SourceState();
  sources_["teleop"] = SourceState();
  sources_["external"] = SourceState();
  sources_["safety"] = SourceState();

  const double period = 1.0 / std::max(1.0, publish_rate_hz_);
  timer_ = nh_.createTimer(ros::Duration(period), &CmdVelMuxNode::timerCallback, this);
}

void CmdVelMuxNode::fixedRouteCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  updateSourceCommand("fixed_route", *msg);
}

void CmdVelMuxNode::teleopCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  updateSourceCommand("teleop", *msg);
}

void CmdVelMuxNode::externalCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  updateSourceCommand("external", *msg);
}

void CmdVelMuxNode::safetyCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  updateSourceCommand("safety", *msg);
}

void CmdVelMuxNode::estopCallback(const std_msgs::Bool::ConstPtr& msg) {
  estop_active_ = msg->data;
  std_msgs::Bool state;
  state.data = estop_active_;
  estop_state_pub_.publish(state);
  if (estop_active_) {
    publishStop("emergency_stop");
  }
}

void CmdVelMuxNode::timerCallback(const ros::TimerEvent& event) {
  const ros::Time now = ros::Time::now();
  const double dt = std::max(1e-3, (event.current_real - event.last_real).toSec());

  if (estop_active_) {
    publishStop("emergency_stop");
    return;
  }

  const char* priority_order[] = {"safety", "external", "teleop", "fixed_route"};
  for (const char* source_name : priority_order) {
    if (!sourceActive(source_name, now)) {
      continue;
    }
    const geometry_msgs::Twist clamped = clampTwist(sources_[source_name].twist, dt);
    publishSelected(clamped, source_name);
    return;
  }

  publishStop("timeout_stop");
}

void CmdVelMuxNode::updateSourceCommand(const std::string& source_name, const geometry_msgs::Twist& msg) {
  SourceState& source = sources_[source_name];
  source.twist = msg;
  source.stamp = ros::Time::now();
  source.has_msg = true;
}

bool CmdVelMuxNode::sourceActive(const std::string& source_name, const ros::Time& now) const {
  const auto it = sources_.find(source_name);
  if (it == sources_.end() || !it->second.has_msg) {
    return false;
  }
  return (now - it->second.stamp).toSec() <= cmd_timeout_sec_;
}

geometry_msgs::Twist CmdVelMuxNode::clampTwist(const geometry_msgs::Twist& input, double dt) const {
  geometry_msgs::Twist output = input;
  output.linear.x = std::max(-max_linear_vel_, std::min(max_linear_vel_, output.linear.x));
  output.linear.y = std::max(-max_linear_vel_, std::min(max_linear_vel_, output.linear.y));
  output.angular.z = std::max(-max_angular_vel_, std::min(max_angular_vel_, output.angular.z));

  const double linear_step = max_linear_accel_ * dt;
  const double angular_step = max_angular_accel_ * dt;

  auto clampStep = [](double target, double last, double step) {
    const double delta = target - last;
    if (std::fabs(delta) <= step) {
      return target;
    }
    return last + std::copysign(step, delta);
  };

  output.linear.x = clampStep(output.linear.x, last_output_.linear.x, linear_step);
  output.linear.y = clampStep(output.linear.y, last_output_.linear.y, linear_step);
  output.angular.z = clampStep(output.angular.z, last_output_.angular.z, angular_step);
  return output;
}

void CmdVelMuxNode::publishSelected(const geometry_msgs::Twist& twist, const std::string& source_name) {
  last_output_ = twist;
  cmd_vel_pub_.publish(twist);

  std_msgs::String source_msg;
  source_msg.data = source_name;
  selected_source_pub_.publish(source_msg);
}

void CmdVelMuxNode::publishStop(const std::string& reason) {
  geometry_msgs::Twist stop_cmd;
  last_output_ = stop_cmd;
  cmd_vel_pub_.publish(stop_cmd);

  std_msgs::String source_msg;
  source_msg.data = reason;
  selected_source_pub_.publish(source_msg);
}

}  // namespace motion_planner
