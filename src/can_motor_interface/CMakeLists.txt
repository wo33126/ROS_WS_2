cmake_minimum_required(VERSION 3.0.2)
project(can_motor_interface)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs can_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(can_interface_node
  src/can_interface_node.cpp
)

add_dependencies(can_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(can_interface_node
  ${catkin_LIBRARIES}
  pthread
)

install(TARGETS can_interface_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
