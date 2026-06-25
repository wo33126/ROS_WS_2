cmake_minimum_required(VERSION 3.0.2)
project(arm_interface)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_controller_node
  src/arm_controller_node.cpp
)

add_dependencies(arm_controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(arm_controller_node
  ${catkin_LIBRARIES}
  pthread
)

install(TARGETS arm_controller_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
