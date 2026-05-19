#include "motion_planner/base_odometry_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "base_odometry_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  motion_planner::BaseOdometryNode node(nh, pnh);
  ros::spin();
  return 0;
}
