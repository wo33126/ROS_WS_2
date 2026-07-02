cmake_minimum_required(VERSION 3.0.2)
project(fine_tuning)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 微调节点 ─────────────────────────────────────────────
add_executable(fine_tuning_node
  src/fine_tuning_node.cpp
)
add_dependencies(fine_tuning_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(fine_tuning_node
  ${catkin_LIBRARIES}
)

install(TARGETS fine_tuning_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
