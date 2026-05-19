#include "motion_planner/cmd_vel_mux_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "cmd_vel_mux_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  motion_planner::CmdVelMuxNode node(nh, pnh);
  ros::spin();
  return 0;
}
