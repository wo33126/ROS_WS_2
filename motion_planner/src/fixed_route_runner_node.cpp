#include "motion_planner/fixed_route_runner.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "fixed_route_runner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  motion_planner::FixedRouteRunner node(nh, pnh);
  ros::spin();
  return 0;
}
