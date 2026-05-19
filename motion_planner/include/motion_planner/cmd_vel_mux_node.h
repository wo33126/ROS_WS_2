#pragma once

#include <map>
#include <string>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace motion_planner {

class CmdVelMuxNode {
 public:
  CmdVelMuxNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);

 private:
  void fixedRouteCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void teleopCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void externalCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void safetyCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void estopCallback(const std_msgs::Bool::ConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);
  void updateSourceCommand(const std::string& source_name, const geometry_msgs::Twist& msg);
  bool sourceActive(const std::string& source_name, const ros::Time& now) const;
  geometry_msgs::Twist clampTwist(const geometry_msgs::Twist& input, double dt) const;
  void publishSelected(const geometry_msgs::Twist& twist, const std::string& source_name);
  void publishStop(const std::string& reason);

 private:
  struct SourceState {
    geometry_msgs::Twist twist;
    ros::Time stamp;
    bool has_msg;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber fixed_route_sub_;
  ros::Subscriber teleop_sub_;
  ros::Subscriber external_sub_;
  ros::Subscriber safety_sub_;
  ros::Subscriber estop_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher selected_source_pub_;
  ros::Publisher estop_state_pub_;
  ros::Timer timer_;

  std::map<std::string, SourceState> sources_;
  geometry_msgs::Twist last_output_;

  double publish_rate_hz_;
  double cmd_timeout_sec_;
  double max_linear_vel_;
  double max_angular_vel_;
  double max_linear_accel_;
  double max_angular_accel_;
  bool estop_active_;
};

}  // namespace motion_planner
