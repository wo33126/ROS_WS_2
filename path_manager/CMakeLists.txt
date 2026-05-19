cmake_minimum_required(VERSION 3.0.2)
project(path_manager)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

# ── 自定义消息 ──────────────────────────────────────────────
add_message_files(
  FILES
  PathPoint.msg
)

# ── 自定义服务 ──────────────────────────────────────────────
add_service_files(
  FILES
  LoadPath.srv
  SelectPath.srv
)

# ── dynamic_reconfigure ────────────────────────────────────
generate_dynamic_reconfigure_options(
  cfg/PathManager.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs nav_msgs geometry_msgs message_runtime dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 可执行文件 ─────────────────────────────────────────────
add_executable(path_manager_node
  src/path_manager_node.cpp
)
add_dependencies(path_manager_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(path_manager_node
  ${catkin_LIBRARIES}
)

# ── 路径跟踪节点 ─────────────────────────────────────────────
add_executable(path_tracker_node
  src/path_tracker_node.cpp
)
add_dependencies(path_tracker_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(path_tracker_node
  ${catkin_LIBRARIES}
)

install(TARGETS path_manager_node path_tracker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
