cmake_minimum_required(VERSION 3.0.2)
project(robot_navigation)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

# ── 依赖 ───────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_generation
  path_manager
)

# ── 消息定义 ───────────────────────────────────────────────
add_message_files(
  FILES
  QrResult.msg
)

# ── 服务定义 ───────────────────────────────────────────────
add_service_files(
  FILES
  OpenMedicineBox.srv
  ArmPlaceMedicine.srv
  Speak.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    message_runtime
    path_manager
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 二维码/条形码检测节点 ──────────────────────────────────
add_executable(barcode_qr_detector_node
  src/barcode_qr_detector.cpp
)
add_dependencies(barcode_qr_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(barcode_qr_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  zbar
)

# ── 任务状态机节点 ─────────────────────────────────────────
add_executable(mission_controller_node
  src/mission_controller.cpp
)
add_dependencies(mission_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mission_controller_node
  ${catkin_LIBRARIES}
)

# ── 药箱开启控制节点 ──────────────────────────────────────
add_executable(open_medicine_box_node
  src/open_medicine_box.cpp
)
add_dependencies(open_medicine_box_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(open_medicine_box_node
  ${catkin_LIBRARIES}
)

# ── 显示屏节点 ─────────────────────────────────────────────
add_executable(display_node
  src/display_node.cpp
)
add_dependencies(display_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(display_node
  ${catkin_LIBRARIES}
)

# ── 启动信号转发节点 ──────────────────────────────────────
add_executable(start_signal_pub_node
  src/start_signal_pub.cpp
)
add_dependencies(start_signal_pub_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(start_signal_pub_node
  ${catkin_LIBRARIES}
)

# ── 安装 ───────────────────────────────────────────────────
install(TARGETS
  barcode_qr_detector_node
  mission_controller_node
  open_medicine_box_node
  display_node
  start_signal_pub_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
