#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace motion_planner {

struct RouteSegment {
  std::string name;
  double vx;
  double vy;
  double omega;
  double duration;
  double target_distance;
  double target_heading;
  bool use_distance_target;
  bool use_heading_target;
};

class FixedRouteRunner {
 public:
  FixedRouteRunner(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~FixedRouteRunner();

 private:
  bool loadRoute();
  bool parseSegments(const XmlRpc::XmlRpcValue& route_list);
  void timerCallback(const ros::TimerEvent& event);
  void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg);
  void controlCallback(const std_msgs::String::ConstPtr& msg);
  void holdCallback(const std_msgs::Bool::ConstPtr& msg);
  void publishCurrentCommand();
  void publishStop();
  void publishStatus(const std::string& status);
  void publishCurrentSegment();
  void publishStateFlags();
  void startRoute(bool restart_from_beginning);
  void stopRoute(const std::string& reason);
  void pauseRoute(const std::string& reason);
  void resumeRoute(const std::string& reason);
  bool normalizeAndHandleCommand(const std::string& command_raw);
  bool segmentReachedGoal(const RouteSegment& segment) const;
  void advanceSegment();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher status_pub_;
  ros::Publisher current_segment_pub_;
  ros::Publisher running_pub_;
  ros::Publisher paused_pub_;
  ros::Publisher hold_pub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber control_sub_;
  ros::Subscriber hold_sub_;
  ros::Timer timer_;

  std::vector<RouteSegment> segments_;
  size_t current_segment_index_;
  ros::Time segment_start_time_;
  ros::Time pause_time_;
  geometry_msgs::Pose2D current_pose_;
  geometry_msgs::Pose2D segment_start_pose_;
  bool has_pose_;

  double publish_rate_hz_;
  double start_delay_sec_;
  bool loop_route_;
  bool auto_start_;
  bool stop_between_segments_;
  bool publish_zero_when_paused_;
  bool is_running_;
  bool finished_;
  bool is_paused_;
  bool external_hold_active_;
};

}  // namespace motion_planner
