cmake_minimum_required(VERSION 3.0.2)
project(motion_planner)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(motion_kinematics
  src/kinematics.cpp
)
add_dependencies(motion_kinematics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(motion_kinematics
  ${catkin_LIBRARIES}
)

add_executable(planner_node
  src/planner_node.cpp
)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(planner_node
  motion_kinematics
  ${catkin_LIBRARIES}
)

add_executable(twist_test_node
  src/twist_test_node.cpp
)
add_dependencies(twist_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(twist_test_node
  ${catkin_LIBRARIES}
)

add_executable(fixed_route_runner_node
  src/fixed_route_runner.cpp
  src/fixed_route_runner_node.cpp
)
add_dependencies(fixed_route_runner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(fixed_route_runner_node
  ${catkin_LIBRARIES}
)

add_executable(base_odometry_node
  src/base_odometry_node.cpp
  src/base_odometry_main.cpp
)
add_dependencies(base_odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(base_odometry_node
  ${catkin_LIBRARIES}
)

add_executable(cmd_vel_mux_node
  src/cmd_vel_mux_node.cpp
  src/cmd_vel_mux_main.cpp
)
add_dependencies(cmd_vel_mux_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(cmd_vel_mux_node
  ${catkin_LIBRARIES}
)

install(TARGETS planner_node twist_test_node fixed_route_runner_node base_odometry_node cmd_vel_mux_node motion_kinematics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
