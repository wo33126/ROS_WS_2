#include "motion_planner/fixed_route_runner.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace motion_planner {

namespace {

double getRequiredDouble(const XmlRpc::XmlRpcValue& value, const std::string& key) {
  if (!value.hasMember(key)) {
    throw std::runtime_error("missing key: " + key);
  }

  const XmlRpc::XmlRpcValue& field = value[key];
  if (field.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(field);
  }
  if (field.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(field);
  }

  throw std::runtime_error("key is not numeric: " + key);
}

double getOptionalDouble(const XmlRpc::XmlRpcValue& value, const std::string& key, double fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }

  const XmlRpc::XmlRpcValue& field = value[key];
  if (field.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(field);
  }
  if (field.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(field);
  }
  return fallback;
}

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::string getOptionalString(const XmlRpc::XmlRpcValue& value, const std::string& key, const std::string& fallback) {
  if (!value.hasMember(key)) {
    return fallback;
  }
  if (value[key].getType() != XmlRpc::XmlRpcValue::TypeString) {
    return fallback;
  }
  return static_cast<std::string>(value[key]);
}

std::string normalizeCommand(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char c) {
                return std::isspace(c) != 0;
              }),
              value.end());
  return value;
}

}  // namespace

FixedRouteRunner::FixedRouteRunner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      current_segment_index_(0),
      publish_rate_hz_(20.0),
      start_delay_sec_(1.0),
      loop_route_(false),
      auto_start_(true),
      stop_between_segments_(false),
      publish_zero_when_paused_(true),
      is_running_(false),
      finished_(false),
      is_paused_(false),
      external_hold_active_(false),
      has_pose_(false) {
  pnh_.param("publish_rate_hz", publish_rate_hz_, publish_rate_hz_);
  pnh_.param("start_delay_sec", start_delay_sec_, start_delay_sec_);
  pnh_.param("loop_route", loop_route_, loop_route_);
  pnh_.param("auto_start", auto_start_, auto_start_);
  pnh_.param("stop_between_segments", stop_between_segments_, stop_between_segments_);
  pnh_.param("publish_zero_when_paused", publish_zero_when_paused_, publish_zero_when_paused_);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_fixed_route", 10);
  status_pub_ = nh_.advertise<std_msgs::String>("/fixed_route/status", 10, true);
  current_segment_pub_ = nh_.advertise<std_msgs::String>("/fixed_route/current_segment", 10, true);
  running_pub_ = nh_.advertise<std_msgs::Bool>("/fixed_route/is_running", 10, true);
  paused_pub_ = nh_.advertise<std_msgs::Bool>("/fixed_route/is_paused", 10, true);
  hold_pub_ = nh_.advertise<std_msgs::Bool>("/fixed_route/external_hold_active", 10, true);
  pose_sub_ = nh_.subscribe("/base_pose2d", 20, &FixedRouteRunner::poseCallback, this);
  control_sub_ = nh_.subscribe("/fixed_route/control", 10, &FixedRouteRunner::controlCallback, this);
  hold_sub_ = nh_.subscribe("/fixed_route/hold", 10, &FixedRouteRunner::holdCallback, this);

  loadRoute();

  if (auto_start_ && !segments_.empty()) {
    startRoute(true);
  } else if (segments_.empty()) {
    finished_ = true;
    publishStatus("route is empty");
  } else {
    publishStatus("route loaded, auto_start disabled");
  }

  publishStateFlags();
  publishCurrentSegment();

  const double period = 1.0 / std::max(1.0, publish_rate_hz_);
  timer_ = nh_.createTimer(ros::Duration(period), &FixedRouteRunner::timerCallback, this);
}

FixedRouteRunner::~FixedRouteRunner() {
  publishStop();
}

bool FixedRouteRunner::loadRoute() {
  XmlRpc::XmlRpcValue route_list;
  if (!pnh_.getParam("route", route_list)) {
    ROS_ERROR("Missing fixed route parameter: ~route");
    return false;
  }

  try {
    return parseSegments(route_list);
  } catch (const std::exception& ex) {
    ROS_ERROR("Failed to parse fixed route: %s", ex.what());
    segments_.clear();
    return false;
  }
}

bool FixedRouteRunner::parseSegments(const XmlRpc::XmlRpcValue& route_list) {
  if (route_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    throw std::runtime_error("route must be a list");
  }

  segments_.clear();
  for (int i = 0; i < route_list.size(); ++i) {
    const XmlRpc::XmlRpcValue& item = route_list[i];
    if (item.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
      throw std::runtime_error("route item must be a map");
    }

    RouteSegment segment;
    segment.name = getOptionalString(item, "name", "segment_" + std::to_string(i));
    segment.vx = getRequiredDouble(item, "vx");
    segment.vy = getRequiredDouble(item, "vy");
    segment.omega = getRequiredDouble(item, "omega");
    segment.duration = getRequiredDouble(item, "duration");
    segment.target_distance = getOptionalDouble(item, "target_distance", 0.0);
    segment.target_heading = getOptionalDouble(item, "target_heading", 0.0);
    segment.use_distance_target = item.hasMember("target_distance");
    segment.use_heading_target = item.hasMember("target_heading");

    if (segment.duration <= 0.0) {
      throw std::runtime_error("duration must be > 0 for " + segment.name);
    }

    segments_.push_back(segment);
  }

  ROS_INFO("Loaded fixed route with %zu segments.", segments_.size());
  return true;
}

void FixedRouteRunner::poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
  current_pose_ = *msg;
  has_pose_ = true;
}

void FixedRouteRunner::timerCallback(const ros::TimerEvent&) {
  if (!is_running_ || finished_ || segments_.empty()) {
    if (publish_zero_when_paused_) {
      publishStop();
    }
    return;
  }

  if (is_paused_ || external_hold_active_) {
    if (publish_zero_when_paused_) {
      publishStop();
    }
    return;
  }

  const ros::Time now = ros::Time::now();
  if (now < segment_start_time_) {
    publishStop();
    return;
  }

  const RouteSegment& segment = segments_[current_segment_index_];
  const double elapsed = (now - segment_start_time_).toSec();

  if (segmentReachedGoal(segment) || elapsed >= segment.duration) {
    advanceSegment();
    return;
  }

  publishCurrentCommand();
}

void FixedRouteRunner::publishCurrentCommand() {
  if (current_segment_index_ >= segments_.size()) {
    publishStop();
    return;
  }

  const RouteSegment& segment = segments_[current_segment_index_];
  geometry_msgs::Twist cmd;
  cmd.linear.x = segment.vx;
  cmd.linear.y = segment.vy;
  cmd.angular.z = segment.omega;
  cmd_vel_pub_.publish(cmd);
}

void FixedRouteRunner::publishStop() {
  geometry_msgs::Twist cmd;
  cmd_vel_pub_.publish(cmd);
}

void FixedRouteRunner::publishStatus(const std::string& status) {
  std_msgs::String msg;
  msg.data = status;
  status_pub_.publish(msg);
}

void FixedRouteRunner::publishCurrentSegment() {
  std_msgs::String msg;
  if (segments_.empty() || current_segment_index_ >= segments_.size()) {
    msg.data = "none";
  } else {
    std::ostringstream oss;
    oss << current_segment_index_ << ":" << segments_[current_segment_index_].name;
    msg.data = oss.str();
  }
  current_segment_pub_.publish(msg);
}

void FixedRouteRunner::publishStateFlags() {
  std_msgs::Bool running_msg;
  running_msg.data = is_running_ && !finished_;
  running_pub_.publish(running_msg);

  std_msgs::Bool paused_msg;
  paused_msg.data = is_paused_;
  paused_pub_.publish(paused_msg);

  std_msgs::Bool hold_msg;
  hold_msg.data = external_hold_active_;
  hold_pub_.publish(hold_msg);
}

void FixedRouteRunner::startRoute(bool restart_from_beginning) {
  if (segments_.empty()) {
    publishStatus("cannot start: route is empty");
    return;
  }

  if (restart_from_beginning) {
    current_segment_index_ = 0;
    finished_ = false;
  }

  is_running_ = true;
  is_paused_ = false;
  segment_start_time_ = ros::Time::now() + ros::Duration(start_delay_sec_);
  segment_start_pose_ = current_pose_;
  publishCurrentSegment();
  publishStateFlags();
  publishStatus(restart_from_beginning ? "route start from beginning" : "route start");
}

void FixedRouteRunner::stopRoute(const std::string& reason) {
  is_running_ = false;
  is_paused_ = false;
  finished_ = true;
  publishStop();
  publishStateFlags();
  publishCurrentSegment();
  publishStatus(reason);
}

void FixedRouteRunner::pauseRoute(const std::string& reason) {
  if (!is_running_ || is_paused_) {
    return;
  }

  is_paused_ = true;
  pause_time_ = ros::Time::now();
  if (publish_zero_when_paused_) {
    publishStop();
  }
  publishStateFlags();
  publishStatus(reason);
}

void FixedRouteRunner::resumeRoute(const std::string& reason) {
  if (!is_running_ || !is_paused_) {
    return;
  }

  const ros::Duration paused_duration = ros::Time::now() - pause_time_;
  segment_start_time_ += paused_duration;
  is_paused_ = false;
  publishStateFlags();
  publishStatus(reason);
}

void FixedRouteRunner::controlCallback(const std_msgs::String::ConstPtr& msg) {
  normalizeAndHandleCommand(msg->data);
}

void FixedRouteRunner::holdCallback(const std_msgs::Bool::ConstPtr& msg) {
  const bool new_hold = msg->data;
  if (new_hold == external_hold_active_) {
    return;
  }

  external_hold_active_ = new_hold;
  publishStateFlags();
  if (external_hold_active_) {
    if (publish_zero_when_paused_) {
      publishStop();
    }
    publishStatus("external hold enabled");
  } else {
    publishStatus("external hold released");
  }
}

bool FixedRouteRunner::normalizeAndHandleCommand(const std::string& command_raw) {
  const std::string command = normalizeCommand(command_raw);

  if (command == "start") {
    startRoute(false);
    return true;
  }
  if (command == "restart") {
    startRoute(true);
    return true;
  }
  if (command == "stop") {
    stopRoute("route stopped by command");
    return true;
  }
  if (command == "pause") {
    pauseRoute("route paused by command");
    return true;
  }
  if (command == "resume") {
    resumeRoute("route resumed by command");
    return true;
  }
  if (command == "next" || command == "skip") {
    if (is_running_ && !finished_) {
      advanceSegment();
    }
    return true;
  }

  publishStatus("unknown control command: " + command_raw);
  return false;
}

bool FixedRouteRunner::segmentReachedGoal(const RouteSegment& segment) const {
  if (!has_pose_) {
    return false;
  }

  if (segment.use_distance_target) {
    const double dx = current_pose_.x - segment_start_pose_.x;
    const double dy = current_pose_.y - segment_start_pose_.y;
    const double traveled = std::sqrt(dx * dx + dy * dy);
    if (traveled >= std::fabs(segment.target_distance)) {
      return true;
    }
  }

  if (segment.use_heading_target) {
    const double heading_delta = std::fabs(normalizeAngle(current_pose_.theta - segment_start_pose_.theta));
    if (heading_delta >= std::fabs(segment.target_heading)) {
      return true;
    }
  }

  return false;
}

void FixedRouteRunner::advanceSegment() {
  if (segments_.empty() || current_segment_index_ >= segments_.size()) {
    stopRoute("route finished");
    return;
  }

  const RouteSegment& finished_segment = segments_[current_segment_index_];
  std::ostringstream oss;
  oss << "finished segment " << current_segment_index_ << " (" << finished_segment.name << ")";
  publishStatus(oss.str());

  if (stop_between_segments_) {
    publishStop();
  }

  ++current_segment_index_;
  if (current_segment_index_ >= segments_.size()) {
    if (loop_route_) {
      current_segment_index_ = 0;
      segment_start_time_ = ros::Time::now();
      segment_start_pose_ = current_pose_;
      publishStatus("route loop restart");
      publishCurrentSegment();
      publishStateFlags();
      return;
    }

    finished_ = true;
    is_running_ = false;
    is_paused_ = false;
    publishStop();
    publishStateFlags();
    publishCurrentSegment();
    publishStatus("route finished");
    ROS_INFO("Fixed route finished.");
    return;
  }

  segment_start_time_ = ros::Time::now();
  segment_start_pose_ = current_pose_;
  const RouteSegment& next_segment = segments_[current_segment_index_];
  publishCurrentSegment();
  publishStateFlags();
  ROS_INFO("Switch to route segment %zu: %s", current_segment_index_, next_segment.name.c_str());
}

}  // namespace motion_planner
