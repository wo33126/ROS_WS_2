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
  arm_and_gripper
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
    arm_and_gripper
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ── 公共 CAN 通信库 ──────────────────────────────────────────
add_library(can_utils
  src/can_utils.cpp
)
add_dependencies(can_utils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(can_utils
  ${catkin_LIBRARIES}
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
  can_utils
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

# ── VL53L1X 激光测距传感器驱动节点 ─────────────────────────
add_executable(vl53l1x_driver_node
  src/vl53l1x_driver.cpp
)
add_dependencies(vl53l1x_driver_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(vl53l1x_driver_node
  ${catkin_LIBRARIES}
)

# ── 距离安全检测节点 ───────────────────────────────────────
add_executable(distance_safety_node
  src/distance_safety_node.cpp
)
add_dependencies(distance_safety_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(distance_safety_node
  ${catkin_LIBRARIES}
)

# ── 语音播报节点 ─────────────────────────────────────────────
add_executable(speak_node
  src/speak_node.cpp
)
add_dependencies(speak_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(speak_node
  ${catkin_LIBRARIES}
)

# ── 系统健康监控节点 ─────────────────────────────────────────
add_executable(health_monitor_node
  src/health_monitor.cpp
)
add_dependencies(health_monitor_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(health_monitor_node
  ${catkin_LIBRARIES}
)

# ── 视频推流节点 ───────────────────────────────────────────
add_executable(video_stream_node
  src/video_stream_node.cpp
)
add_dependencies(video_stream_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(video_stream_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# ── 双向音频聊天节点 ──────────────────────────────────────
add_executable(audio_chat_node
  src/audio_chat_node.cpp
)
add_dependencies(audio_chat_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(audio_chat_node
  ${catkin_LIBRARIES}
)

# ── 物理启动按钮节点 (GPIO) ────────────────────────────────
add_executable(start_button_node
  src/start_button_node.cpp
)
add_dependencies(start_button_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(start_button_node
  ${catkin_LIBRARIES}
)

# ── 急停按钮节点 (GPIO) ────────────────────────────────────
add_executable(emergency_stop_node
  src/emergency_stop_node.cpp
)
add_dependencies(emergency_stop_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(emergency_stop_node
  ${catkin_LIBRARIES}
)

# ── 安装 ───────────────────────────────────────────────────
install(TARGETS
  can_utils
  barcode_qr_detector_node
  mission_controller_node
  open_medicine_box_node
  display_node
  start_signal_pub_node
  vl53l1x_driver_node
  distance_safety_node
  speak_node
  health_monitor_node
  video_stream_node
  audio_chat_node
  start_button_node
  emergency_stop_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
