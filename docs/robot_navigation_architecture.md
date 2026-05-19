# 机器人导航系统架构说明

本文档说明本工作空间中所有功能包的作用、依赖关系、启动顺序及整体架构设计思路。

---

## 1. 架构概览

系统采用**分层架构**设计，从底层硬件通信到顶层业务逻辑共分为 8 层：

```
┌─────────────────────────────────────────────────────────────┐
│  第8层: 业务层     arm_and_gripper    药品摆放动作序列      │
├─────────────────────────────────────────────────────────────┤
│  第7层: 舵机层     servo_controller   舵机PWM控制           │
├─────────────────────────────────────────────────────────────┤
│  第6层: 微调层     fine_tuning        VL53L1X终点距离微调   │
├─────────────────────────────────────────────────────────────┤
│  第5层: 跟踪层     path_tracker       路径点跟踪控制         │
├─────────────────────────────────────────────────────────────┤
│  第4层: 路径层     path_manager       路径加载/管理/发布     │
├─────────────────────────────────────────────────────────────┤
│  第3层: 机械臂层   arm_interface      机械臂CAN接口(可选)    │
├─────────────────────────────────────────────────────────────┤
│  第2层: 规划层     motion_planner     运动学+里程计+固定路线  │
├─────────────────────────────────────────────────────────────┤
│  第1层: 通信层     can_motor_interface 底盘CAN(ST信32)       │
└─────────────────────────────────────────────────────────────┘
```

### 数据流简图

```
路径YAML → path_manager → PathPoint[] → path_tracker → /cmd_vel
                                                           ↓
                                                    cmd_vel_mux (多源合并)
                                                           ↓
                                                      planner_node
                                                    (Twist → RPM)
                                                           ↓
                                                   /motor_velocity_cmd
                                                           ↓
                                                   can_interface_node
                                                    (RPM → CAN帧)
                                                           ↓
                                                     STM32 → 电机

到达终点后:
  path_tracker → /arrival_signal → fine_tuning → /cmd_vel (微调)
  fine_tuning → /fine_tuning_done → arm_and_gripper → ServoCommand/CAN
```

---

## 2. 功能包详细说明

### 2.1 `can_motor_interface` — 底盘 CAN 通信接口

| 项目 | 内容 |
|------|------|
| **层级** | 第1层：硬件通信层 |
| **功能** | 与 STM32 电机控制器通过 SocketCAN 通信，下发速度指令，轮询电机状态 |
| **订阅** | `/motor_velocity_cmd` (`std_msgs/Float32MultiArray`, RPM) |
| **发布** | `/motor_state`, `/motor_position_deg`, `/motor_error_deg`, `/motor_temperature_c`, `/motor_current_ma`, `/motor_bus_voltage_mv`, `/motor_status_flags`, `/motor_homing_flags`, `/can_rx` |
| **参数** | `can_device`, `tx_can_id`, `rx_can_id`, `motor_count`, `max_rpm` |
| **前置条件** | CAN 接口已 UP: `sudo ip link set can0 up type can bitrate 1000000` |

### 2.2 `motion_planner` — 运动规划

| 项目 | 内容 |
|------|------|
| **层级** | 第2层：运动规划层 |
| **节点** | `cmd_vel_mux_node`, `base_odometry_node`, `planner_node`, `fixed_route_runner_node`, `twist_test_node` |
| **功能** | Twist → 四轮 RPM 转换（全向轮/麦克纳姆轮运动学）、里程计计算、固定路线执行 |
| **订阅** | `/cmd_vel`（多个源经 mux 合并） |
| **发布** | `/motor_velocity_cmd`, `/odom`, TF |

**子节点说明：**

| 节点 | 作用 |
|------|------|
| `cmd_vel_mux_node` | 多源 `/cmd_vel` 优先级合并（遥控/路径跟踪/微调/固定路线） |
| `base_odometry_node` | 基于电机反馈计算里程计 (`/odom`, TF) |
| `planner_node` | 运动学解算：Twist → 各轮 RPM |
| `fixed_route_runner_node` | 从 YAML 读取固定路线并按顺序发布 `/cmd_vel` |
| `twist_test_node` | Twist 指令测试工具 |

### 2.3 `arm_interface` — 机械臂 CAN 接口

| 项目 | 内容 |
|------|------|
| **层级** | 第3层：机械臂硬件层（可选） |
| **功能** | 通过 CAN 总线与机械臂关节电机通信 |
| **订阅** | `/arm_joint_cmd` (`sensor_msgs/JointState`) |
| **发布** | `/arm_state` |
| **前置条件** | CAN 接口已 UP |

### 2.4 `path_manager` — 路径管理

| 项目 | 内容 |
|------|------|
| **层级** | 第4层：路径管理层 |
| **功能** | 从 YAML 加载多条命名路径，支持运行时选择路径、加载/重载路径，按顺序发布路径点 |
| **服务** | `/path_manager/select_path` (`SelectPath`), `/path_manager/load_path` (`LoadPath`) |
| **发布** | `/path_manager/current_path_point` (`PathPoint`), `/path_manager/path_progress` |
| **参数** | `yaml_file_path`, `publish_rate_hz` |

**路径 YAML 格式**（见 `robot_bringup/config/paths.yaml`）：
```yaml
paths:
  - name: "medicine_rack"
    points:
      - {x: 1.0, y: 0.0, tolerance: 0.05}
      - {x: 1.5, y: 0.5, tolerance: 0.05}
      - {x: 2.0, y: 0.0, tolerance: 0.05}
```

### 2.5 `path_tracker` — 路径跟踪

| 项目 | 内容 |
|------|------|
| **层级** | 第5层：路径跟踪层 |
| **功能** | 订阅路径点，使用比例控制器驱动机器人逐点到达，到达后发布完成信号 |
| **订阅** | `/path_manager/current_path_point` (`PathPoint`), `/odom` |
| **发布** | `/cmd_vel`, `/path_tracker/arrival` |
| **参数** | `kp_linear`, `kp_angular`, `position_tolerance`, `heading_tolerance`, `arrival_hold_time` |

**控制模式**：
- `use_turn_then_move=false`（推荐）：同时控制线速度和角速度，平滑到达
- `use_turn_then_move=true`：先原地转向对准，再直线前进

### 2.6 `fine_tuning` — 终点位置微调

| 项目 | 内容 |
|------|------|
| **层级** | 第6层：微调层 |
| **功能** | 路径跟踪到达后，使用 VL53L1X 距离传感器进行精确位置微调（PID 控制） |
| **订阅** | `/path_tracker/arrival`, `/vl53l1x/distance`（或传感器话题） |
| **发布** | `/cmd_vel` (微调指令), `/fine_tuning_done` |
| **参数** | `target_distance_mm`, `tolerance_mm`, `step_velocity`, `kp_distance` |
| **前置条件** | VL53L1X 距离传感器已连接并发布数据 |

### 2.7 `servo_controller` — 舵机控制

| 项目 | 内容 |
|------|------|
| **层级** | 第7层：舵机控制层 |
| **功能** | 通过 pigpio 库输出 PWM 信号控制两个舵机（夹爪开合） |
| **服务** | `/servo_controller/servo_command` (`ServoCommand`) |
| **参数** | `servo1_gpio`, `servo2_gpio`, PWM 范围, 默认角度 |
| **前置条件** | `sudo pigpiod` 已在后台运行 |

### 2.8 `arm_and_gripper` — 药品摆放控制

| 项目 | 内容 |
|------|------|
| **层级** | 第8层：业务逻辑层 |
| **功能** | 协调机械臂电机（CAN2）和两个舵机，执行药品放置动作序列 |
| **服务** | `/arm_and_gripper/place_medicine` (`PlaceMedicine`) |
| **订阅** | `/fine_tuning_done` |
| **时序** | 舵机1张开 → 等待 → 舵机2张开 → 等待 → 机械臂旋转到位 → 复位 |
| **前置条件** | CAN2 接口已 UP, pigpiod 已运行 |

### 2.9 `robot_bringup` — 基础启动包

| 项目 | 内容 |
|------|------|
| **功能** | 提供基础参数文件（`robot_params.yaml`, `arm_params.yaml`, `paths.yaml`, `fixed_route.yaml`, `path_tracker.yaml`）和基础启动文件 `robot_bringup.launch` |
| **与 robot_navigation 的关系** | `robot_bringup` 是基础版本，`robot_navigation` 是完整导航场景的增强编排版本 |

### 2.10 `robot_navigation` — 完整导航聚合包（本包）

| 项目 | 内容 |
|------|------|
| **层级** | 编排层（跨所有8层） |
| **功能** | 一键启动完整导航系统的所有节点，提供聚合参数文件和统一 launch 入口 |
| **核心文件** | `launch/complete_navigation.launch`, `config/navigation_params.yaml` |
| **本质** | 纯 launch/config 编排包，不含源代码 |

---

## 3. 启动顺序

### 3.1 硬件前置条件

在启动任何 ROS 节点之前，确保以下硬件已就绪：

```bash
# 1. CAN 接口（底盘电机）
sudo ip link set can0 up type can bitrate 1000000

# 2. CAN 接口（机械臂电机，如果需要）
sudo ip link set can1 up type can bitrate 1000000

# 3. pigpio 守护进程（舵机控制）
sudo pigpiod
```

### 3.2 完整启动（推荐）

```bash
# 一键启动所有导航节点
roslaunch robot_navigation complete_navigation.launch

# 带固定路线模式
roslaunch robot_navigation complete_navigation.launch enable_fixed_route:=true

# 带键盘遥控
roslaunch robot_navigation complete_navigation.launch enable_teleop:=true

# 只启动底盘+规划（不含路径/微调/机械臂）
roslaunch robot_navigation complete_navigation.launch \
    enable_path_manager:=false \
    enable_path_tracker:=false \
    enable_fine_tuning:=false \
    enable_servo:=false \
    enable_arm_and_gripper:=false \
    enable_arm:=false
```

### 3.3 分步启动（调试用）

按依赖关系从底层到顶层依次启动：

```bash
# 第1步：底盘通信
roslaunch can_motor_interface can_interface.launch

# 第2步：运动规划
roslaunch motion_planner planner.launch

# 第3步（可选）：机械臂
roslaunch arm_interface arm.launch

# 第4步：路径管理
roslaunch path_manager path_manager.launch

# 第5步：路径跟踪
roslaunch path_manager path_tracker.launch

# 第6步：终点微调
roslaunch fine_tuning fine_tuning.launch

# 第7步：舵机控制
roslaunch arm_and_gripper servo_controller.launch

# 第8步：药品摆放
roslaunch arm_and_gripper arm_and_gripper.launch
```

### 3.4 自动启动顺序保证

`complete_navigation.launch` 内部按如下顺序启动节点（ROS launch 机制保证 XML 中先声明的节点先启动）：

| 序号 | 节点 | 所属包 | 层级 |
|------|------|--------|------|
| 1 | `can_interface_node` | `can_motor_interface` | 通信层 |
| 2 | `cmd_vel_mux_node` | `motion_planner` | 规划层 |
| 3 | `base_odometry_node` | `motion_planner` | 规划层 |
| 4 | `planner_node` | `motion_planner` | 规划层 |
| 5 | `arm_controller_node` | `arm_interface` | 机械臂层 |
| 6 | `path_manager_node` | `path_manager` | 路径层 |
| 7 | `path_tracker_node` | `path_manager` | 跟踪层 |
| 8 | `fine_tuning_node` | `fine_tuning` | 微调层 |
| 9 | `servo_controller_node` | `arm_and_gripper` | 舵机层 |
| 10 | `arm_and_gripper_node` | `arm_and_gripper` | 业务层 |

> **注意**：虽然 launch 文件内按顺序声明，但 ROS 不严格保证节点启动的先后顺序。各节点通过话题订阅机制自然形成运行时依赖链：
> - `path_tracker` 等待 `path_manager` 发布路径点才工作
> - `fine_tuning` 等待 `path_tracker` 发布到达信号才工作
> - `arm_and_gripper` 等待 `fine_tuning` 发布完成信号才工作

---

## 4. 话题与服务接口总览

### 4.1 核心话题

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 多进1出(mux) | 速度指令 |
| `/motor_velocity_cmd` | `std_msgs/Float32MultiArray` | planner→CAN | 四轮目标 RPM |
| `/odom` | `nav_msgs/Odometry` | 发布 | 里程计 |
| `/motor_state` | `std_msgs/Float32MultiArray` | CAN→planner | 电机实时 RPM |
| `/path_manager/current_path_point` | `path_manager/PathPoint` | 发布 | 当前目标路径点 |
| `/path_tracker/arrival` | `std_msgs/Bool` | 发布 | 到达路径终点 |
| `/fine_tuning_done` | `std_msgs/Bool` | 发布 | 微调完成 |

### 4.2 服务

| 服务 | 类型 | 提供者 | 说明 |
|------|------|--------|------|
| `/path_manager/select_path` | `path_manager/SelectPath` | path_manager | 按名称选择路径 |
| `/path_manager/load_path` | `path_manager/LoadPath` | path_manager | 加载新路径 YAML |
| `/servo_controller/servo_command` | `arm_and_gripper/ServoCommand` | servo_controller | 舵机角度命令 |
| `/arm_and_gripper/place_medicine` | `arm_and_gripper/PlaceMedicine` | arm_and_gripper | 触发药品摆放 |

---

## 5. 配置文件说明

### 5.1 配置文件清单

| 文件 | 所属包 | 作用 |
|------|--------|------|
| `config/robot_params.yaml` | `robot_bringup` | 机器人物理参数（轮径、轴距、速度限制） |
| `config/arm_params.yaml` | `robot_bringup` | 机械臂参数（关节数、DH 参数、限位） |
| `config/paths.yaml` | `robot_bringup` | 预定义路径（命名路径点列表） |
| `config/fixed_route.yaml` | `robot_bringup` | 固定路线（速度+持续时间序列） |
| `config/path_tracker.yaml` | `robot_bringup` | 路径跟踪控制器参数 |
| `config/fine_tuning.yaml` | `fine_tuning` | 终点微调参数 |
| `config/servo_params.yaml` | `arm_and_gripper` | 舵机 GPIO 和 PWM 参数 |
| `config/arm_and_gripper.yaml` | `arm_and_gripper` | 机械臂+夹爪控制参数 |
| `config/navigation_params.yaml` | `robot_navigation` | **聚合参数**（汇总以上所有参数） |

### 5.2 参数优先级

当同一参数在多个 YAML 文件中定义时，后加载的覆盖先加载的。`complete_navigation.launch` 中的加载顺序：

```
robot_params.yaml  →  arm_params.yaml  →  navigation_params.yaml
                                                ↓
                                    各节点独立加载的 YAML（覆盖聚合参数）
```

建议：日常调参修改各子包独立的 YAML 文件；`navigation_params.yaml` 作为参考快照。

---

## 6. 典型工作流程

### 6.1 自动导航到药架并放药（完整流程）

```
1. 系统启动 → 所有节点就绪
2. 调用 /path_manager/select_path "medicine_rack"
3. path_manager 按顺序发布路径点
4. path_tracker 跟踪每个路径点，通过 /cmd_vel 控制底盘
5. 到达最后一个路径点 → 发布 /path_tracker/arrival
6. fine_tuning 收到到达信号 → 用 VL53L1X 微调距离
7. 微调完成 → 发布 /fine_tuning_done
8. arm_and_gripper 收到完成信号 → 执行放药动作序列
   - 舵机1张开（释放夹爪1）
   - 舵机2张开（释放夹爪2）
   - 机械臂归位
9. 完成
```

### 6.2 固定路线测试

```bash
roslaunch robot_navigation complete_navigation.launch enable_fixed_route:=true
```
机器人将按 `fixed_route.yaml` 中定义的路线段顺序自动执行。

### 6.3 键盘手动控制

```bash
roslaunch robot_navigation complete_navigation.launch enable_teleop:=true
```
使用键盘 i/j/l/, 等控制机器人移动。

---

## 7. 扩展指南

### 7.1 添加新的路径

编辑 `robot_bringup/config/paths.yaml`，添加新的命名路径：

```yaml
paths:
  - name: "new_location"
    points:
      - {x: 0.0, y: 0.0, tolerance: 0.05}
      - {x: 1.0, y: 1.0, tolerance: 0.05}
      - {x: 2.0, y: 1.5, tolerance: 0.03}
```

### 7.2 添加定位节点

在 `complete_navigation.launch` 中取消 `positioning_node` 的注释，或添加：

```xml
<node pkg="your_positioning_pkg" type="positioning_node"
      name="positioning_node" output="screen"/>
```

定位节点应发布 `/odom` 或提供 TF 变换，用于 path_tracker 的全局定位。

### 7.3 添加新传感器

在 `complete_navigation.launch` 对应层级添加新节点的 `<include>` 或 `<node>` 元素即可。

---

## 8. 故障排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 电机不转 | CAN 接口未 UP | `sudo ip link set can0 up type can bitrate 1000000` |
| 舵机不响应 | pigpiod 未运行 | `sudo pigpiod` |
| path_tracker 不工作 | 无里程计数据 | 检查 `base_odometry_node` 是否正常运行 |
| fine_tuning 卡住 | VL53L1X 无数据 | 检查传感器连接和话题发布 |
| 机械臂不动 | CAN1 未 UP | `sudo ip link set can1 up type can bitrate 1000000` |
| launch 报依赖缺失 | 未编译 | `catkin build robot_navigation` 或 `catkin_make` |

---

## 9. 目录结构总结

```
ros_ws/src/
├── can_motor_interface/     # 第1层: 底盘CAN通信
├── motion_planner/          # 第2层: 运动规划+里程计
├── arm_interface/           # 第3层: 机械臂CAN接口(可选)
├── path_manager/            # 第4-5层: 路径管理+跟踪
├── fine_tuning/             # 第6层: 终点距离微调
├── arm_and_gripper/         # 第7-8层: 舵机+药品摆放
├── robot_bringup/           # 基础启动包+参数文件
├── robot_navigation/        # ★ 完整导航聚合包(本包)
│   ├── launch/
│   │   └── complete_navigation.launch  # 一键启动
│   ├── config/
│   │   └── navigation_params.yaml      # 聚合参数
│   ├── CMakeLists.txt
│   └── package.xml
└── docs/
    ├── 固定路线模块使用说明.md
    └── robot_navigation_architecture.md  # ★ 本文档
```
