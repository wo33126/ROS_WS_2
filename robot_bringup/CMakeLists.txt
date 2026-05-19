cmake_minimum_required(VERSION 3.0.2)
project(robot_bringup)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 确保 path_manager 的消息/服务已生成
add_custom_target(robot_bringup_check DEPENDS path_manager_generate_messages)
