#include "motion_planner/kinematics.h"

#include <cmath>

namespace motion_planner {

Kinematics::Kinematics() : wheel_radius_(0.05), wheel_base_(0.2) {}

void Kinematics::loadFromRos(const ros::NodeHandle& nh, const ros::NodeHandle& pnh) {
  nh.param("/robot/wheel_radius", wheel_radius_, wheel_radius_);
  nh.param("/robot/wheel_base", wheel_base_, wheel_base_);

  pnh.param("wheel_radius", wheel_radius_, wheel_radius_);
  pnh.param("wheel_base", wheel_base_, wheel_base_);

  if (wheel_radius_ <= 1e-6) {
    wheel_radius_ = 0.05;
  }
  if (wheel_base_ < 0.0) {
    wheel_base_ = 0.0;
  }
}

void Kinematics::twistToWheelLinearVelocities(double vx,
                                              double vy,
                                              double omega,
                                              double wheel_linear_vels[4]) const {
  // 四全向轮（前/右/后/左，安装角 0/90/180/270 deg）
  // omega 符号取反以匹配电机实际转向
  // v1 =  vx - omega * L
  // v2 =  vy - omega * L
  // v3 = -vx - omega * L
  // v4 = -vy - omega * L
  wheel_linear_vels[0] = vx - omega * wheel_base_;   // 前
  wheel_linear_vels[1] = vy - omega * wheel_base_;   // 右
  wheel_linear_vels[2] = -vx - omega * wheel_base_;  // 后
  wheel_linear_vels[3] = -vy - omega * wheel_base_;  // 左
}

double Kinematics::linearToRpm(double wheel_linear_vel_mps) const {
  if (wheel_radius_ <= 1e-6) {
    return 0.0;
  }
  return (wheel_linear_vel_mps * 60.0) / (2.0 * M_PI * wheel_radius_);
}

}  // namespace motion_planner
