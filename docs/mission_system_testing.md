# 任务控制系统 — 测试与调试说明

## 文件清单

### 新增/修改的消息和服务

| 文件 | 位置 | 说明 |
|------|------|------|
| `QrResult.msg` | `robot_navigation/msg/` | QR 识别结果：first_bed, first_box, second_bed, second_box |
| `OpenMedicineBox.srv` | `robot_navigation/srv/` | 开药箱服务 (int8 box_id → bool success) |
| `ArmPlaceMedicine.srv` | `robot_navigation/srv/` | 机械臂放置药品 (int8 bed_id → bool success) |
| `Speak.srv` | `robot_navigation/srv/` | 语音播报 (string text → bool success) |
| `ArmPlaceMedicine.srv` | `arm_and_gripper/srv/` | 机械臂放置药品（arm_and_gripper 侧） |

### 新增节点

| 节点 | 源文件 | 功能 |
|------|--------|------|
| `barcode_qr_detector_node` | `robot_navigation/src/barcode_qr_detector.cpp` | 二维码/条形码识别 |
| `mission_controller_node` | `robot_navigation/src/mission_controller.cpp` | 任务状态机 |
| `open_medicine_box_node` | `robot_navigation/src/open_medicine_box.cpp` | 药箱开启控制 |
| `display_node` | `robot_navigation/src/display_node.cpp` | 屏幕显示 |
| `start_signal_pub_node` | `robot_navigation/src/start_signal_pub.cpp` | 启动信号转发 |

### 修改的节点

| 节点 | 文件 | 变更 |
|------|------|------|
| `fine_tuning_node` | `fine_tuning/src/fine_tuning_node.cpp` | 新增 `/fine_tuning/start` 服务（std_srvs::Trigger）|
| `arm_and_gripper_node` | `arm_and_gripper/src/arm_and_gripper_node.cpp` | 新增 `/arm_place_medicine` 服务和推出动作 |

### 配置文件

| 文件 | 说明 |
|------|------|
| `robot_navigation/config/mission_params.yaml` | 任务参数：超时、圆圈坐标、起始区等 |
| `mission_complete.launch` | 一键启动所有节点 |

---

## 编译方法

```bash
# 1. 确保已安装依赖
sudo apt install libzbar-dev ros-noetic-cv-bridge ros-noetic-usb-cam

# 2. 编译
cd ~/ros_ws
catkin_make

# 3. 环境
source devel/setup.bash
```

---

## 测试步骤

### 阶段一：单元测试（无硬件）

```bash
# 1. 启动最小系统（仅核心节点）
roscore

# 2. 测试二维码识别（使用测试图片）
# 发布测试图像话题
rosrun robot_navigation barcode_qr_detector_node

# 3. 测试任务状态机
rosrun robot_navigation mission_controller_node

# 4. 手动触发启动信号
rostopic pub /start_signal std_msgs/Empty "{}"
```

### 阶段二：话题级测试

```bash
# 1. 测试 QR 码识别
# 发布模拟 QR 结果
rostopic pub /qr_result robot_navigation/QrResult \
  "{first_bed: 1, first_box: 1, second_bed: 3, second_box: 3}"

# 2. 测试条形码发布
rostopic pub /barcode_bed1 std_msgs/String "data: '6901234567890'"
rostopic pub /barcode_bed3 std_msgs/String "data: '6909876543210'"

# 3. 测试服务调用
rosservice call /open_medicine_box "box_id: 1"
rosservice call /fine_tuning/start "{}"
rosservice call /arm_place_medicine "bed_id: 1"
rosservice call /speak "text: '1床病人请取药'"

# 4. 监控状态
rostopic echo /mission_finished
rostopic echo /mission_timeout
rostopic echo /robot_display
```

### 阶段三：硬件集成测试

```bash
# 1. 启动完整系统
roslaunch robot_navigation mission_complete.launch

# 2. 或只启动必要节点（关闭摄像头）
roslaunch robot_navigation mission_complete.launch enable_camera:=false enable_arm:=false

# 3. 手动发布启动信号（如果 STM32 未连接）
rostopic pub /start_signal std_msgs/Empty "{}"

# 4. 查看各节点状态
rosnode list
rosnode info /mission_controller_node
```

---

## 调试技巧

### 查看状态机当前状态

```bash
# 查看 mission_controller 日志
rqt_console
# 或
rostopic echo /robot_display
```

### 模拟路径选择

```bash
# 手动调用路径选择（模拟 path_manager）
rosservice call /select_path "path_name: 'nurse_station'"
```

### 模拟微调完成

```bash
# 发布微调完成信号
rostopic pub /fine_tuning_done std_msgs/Bool "data: true"
```

### 检查 CAN 通信

```bash
# 监控 CAN 总线
candump can0
candump can1
```

### 单独测试各节点

```bash
# 测试药箱节点
rosrun robot_navigation open_medicine_box_node _can_device:=can1
# 另一终端：
rosservice call /open_medicine_box "box_id: 1"

# 测试显示节点
rosrun robot_navigation display_node
# 另一终端：
rostopic pub /robot_display std_msgs/String "data: '1床药品: 6901234567890 | 3床药品: 6909876543210'"
```

---

## 常见问题

1. **zbar 未安装**: `sudo apt install libzbar-dev`
2. **cv_bridge 找不到**: 确认 `ros-noetic-cv-bridge` 已安装
3. **CAN 接口不可用**: 确保 `sudo ip link set can0 up type can bitrate 1000000`
4. **服务不可用**: 检查 `mission_complete.launch` 中节点启动顺序
5. **路径选择失败**: 确认 `paths.yaml` 中定义了 `nurse_station`, `bed1_circle`, `bed3_circle`, `HOME` 路径
