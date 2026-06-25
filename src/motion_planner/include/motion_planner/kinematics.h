#pragma once

#include <ros/ros.h>

namespace motion_planner {

class Kinematics {
 public:
  Kinematics();

  void loadFromRos(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  void twistToWheelLinearVelocities(double vx, double vy, double omega, double wheel_linear_vels[4]) const;
  double linearToRpm(double wheel_linear_vel_mps) const;

 private:
  double wheel_radius_;  // m
  double wheel_base_;    // m, 轮心到车体中心距离 L
};

}  // namespace motion_planner
