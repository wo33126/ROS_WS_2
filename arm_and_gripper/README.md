# arm_and_gripper — 药品摆放控制包

## 概述

在微调（fine_tuning）完成后，自动执行药品摆放动作序列，控制：
- **机械臂电机**：Y42 步进电机（X固件），通过 STM32 经 CAN2（地址5）以位置模式控制旋转角度
- **舵机1 / 舵机2**：SG90 小型舵机，通过树莓派 GPIO + pigpio 库的 PWM 控制

## 架构

```
/fine_tuning_done (std_msgs/Bool)
        │
        ▼
┌─────────────────────────┐     /servo_command (ServoCommand.srv)
│  arm_and_gripper_node   │ ──────────────────────────────────────►  servo_controller_node
│  - 订阅微调完成信号      │                                            - pigpio PWM 控制
│  - /place_medicine 服务 │                                            - GPIO18 → 舵机1
│  - CAN2 → Y42 电机      │                                            - GPIO19 → 舵机2
└─────────────────────────┘
```

## 文件结构

```
arm_and_gripper/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── arm_and_gripper.yaml    # 机械臂+夹爪参数
│   └── servo_params.yaml       # 舵机参数
├── include/arm_and_gripper/
│   ├── arm_and_gripper_controller.h
│   └── servo_controller.h
├── launch/
│   ├── arm_and_gripper.launch  # 主启动（包含舵机+机械臂）
│   └── servo_controller.launch # 仅舵机
├── scripts/
│   └── pigpiod_start.sh        # pigpiod 守护进程启动脚本
├── src/
│   ├── arm_and_gripper_node.cpp
│   └── servo_controller_node.cpp
└── srv/
    ├── PlaceMedicine.srv       # 药品摆放服务
    └── ServoCommand.srv        # 舵机控制服务
```

## 依赖

| 依赖 | 说明 |
|------|------|
| `roscpp` | ROS C++ 客户端 |
| `std_msgs` | 标准消息 |
| `pigpio` | 树莓派 GPIO/PWM 库 (`libpigpio-dev`) |
| SocketCAN | Linux CAN 总线 (`can-utils`) |

安装 pigpio：
```bash
sudo apt install pigpio libpigpio-dev
```

## 硬件连接

| 硬件 | 接口 | 说明 |
|------|------|------|
| Y42 机械臂电机 | CAN2 (can1) | STM32 控制，CAN ID: 0x205（地址5） |
| 舵机1 (SG90) | GPIO18 (BCM) | 物理引脚12，PWM信号线 |
| 舵机2 (SG90) | GPIO19 (BCM) | 物理引脚35，PWM信号线 |
| 舵机电源 | 5V / GND | 从树莓派或外部电源取电（注意电流） |

> ⚠️ **注意**：SG90 舵机峰值电流可达 700mA。两个舵机同时工作时建议使用外部 5V 电源，不要从树莓派 5V 引脚直接取电。

## 使用方法

### 1. 前置准备

```bash
# 启动 CAN2 接口
sudo ip link set can1 up type can bitrate 1000000

# 启动 pigpiod 守护进程
sudo pigpiod
```

### 2. 编译

```bash
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

### 3. 单独启动（调试用）

```bash
# 仅启动舵机控制节点
roslaunch arm_and_gripper servo_controller.launch

# 启动完整药品摆放控制
roslaunch arm_and_gripper arm_and_gripper.launch
```

### 4. 与主系统集成

在 `robot_bringup.launch` 中设置：
```xml
<arg name="enable_arm_and_gripper" default="true"/>
<arg name="enable_fine_tuning" default="true"/>
```

```bash
roslaunch robot_bringup robot_bringup.launch \
    enable_arm_and_gripper:=true \
    enable_fine_tuning:=true
```

### 5. 手动触发药品摆放

无需等待微调完成，直接调用服务：

```bash
# 使用默认参数
rosservice call /place_medicine "{arm_angle: -1.0, servo1_open_angle: -1.0, servo2_open_angle: -1.0, reset_arm: true}"

# 指定角度
rosservice call /place_medicine "{arm_angle: 45.0, servo1_open_angle: 90.0, servo2_open_angle: 120.0, reset_arm: true}"
```

### 6. 单独控制舵机

```bash
# 舵机1旋转到90度
rosservice call /servo_command "{servo_id: 1, angle: 90.0}"

# 舵机2旋转到45度
rosservice call /servo_command "{servo_id: 2, angle: 45.0}"
```

## 动作序列

当 `/fine_tuning_done` 变为 `true` 或 `/place_medicine` 服务被调用时：

| 步骤 | 动作 | 说明 |
|------|------|------|
| 1 | 机械臂 → 目标角度 | 通过 CAN2 发送位置指令到 Y42 电机 |
| 2 | 舵机1张开 | 延时 0.5s |
| 3 | 舵机2张开 | 延时 1.0s |
| 4 | 舵机1闭合 | 抓取/释放完成 |
| 5 | 机械臂复位 (可选) | 回到 0° |

## ROS 接口

### 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/fine_tuning_done` | `std_msgs/Bool` | 微调完成信号（由 `fine_tuning_node` 发布） |

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/place_medicine` | `PlaceMedicine.srv` | 手动触发药品摆放动作 |
| `/servo_command` | `ServoCommand.srv` | 单独控制某个舵机角度 |

### PlaceMedicine.srv

```yaml
# 请求
float32 arm_angle          # 目标角度(<0 使用默认)
float32 servo1_open_angle
float32 servo2_open_angle
bool reset_arm
---
# 响应
bool success
string message
```

### ServoCommand.srv

```yaml
# 请求
uint8 servo_id   # 1 或 2
float32 angle    # 0~180 度
---
# 响应
bool success
string message
```

## 配置参数

### arm_and_gripper.yaml

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `can_device` | `can1` | CAN 接口 |
| `arm_can_id` | `0x205` | 电机 CAN ID |
| `arm_angle_scale` | `2.78` | 角度→counts 换算 |
| `default_arm_angle` | `90.0` | 默认机械臂角度 |
| `post_servo1_delay_s` | `0.5` | 舵机1动作后延时 |
| `post_servo2_delay_s` | `1.0` | 舵机2动作后延时 |

### servo_params.yaml

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `servo1_gpio` | `18` | 舵机1 GPIO (BCM) |
| `servo2_gpio` | `19` | 舵机2 GPIO (BCM) |
| `servo1_min_pw_us` | `500` | 0° 脉宽 |
| `servo1_max_pw_us` | `2500` | 180° 脉宽 |

## CAN 协议

机械臂电机 (Y42, CAN ID 0x205) 使用 8 字节标准帧：

| 字节 | 含义 |
|------|------|
| [0] | 指令码: `0x10` (位置模式) |
| [1] | 电机索引: `0` |
| [2-5] | 目标位置 (int32, little-endian, 单位: counts) |
| [6-7] | 保留 |

角度到计数值换算：`counts = angle × arm_angle_scale`（默认 1000 counts = 360°）

## 权限说明

### pigpio
`pigpio` 需要 root 权限访问 `/dev/mem`。有两种方式：

1. **守护进程模式（推荐）**：
   ```bash
   sudo pigpiod         # 手动启动
   # 或在 launch 中通过 sudo 自动启动
   ```

2. **sudo 运行节点**：
   ```xml
   <node ... launch-prefix="sudo -E"/>
   ```

### CAN
CAN 接口需要 `ip link set` 配置，通常由系统启动脚本或 `udev` 规则自动完成。

## 故障排查

| 问题 | 解决方法 |
|------|----------|
| `pigpio 初始化失败` | 确认 `sudo pigpiod` 已运行 |
| CAN 写入失败 | 确认 `can1` 已 UP: `ip link show can1` |
| 舵机不动作 | 检查 GPIO 连接和外部电源 |
| `/servo_command` 不可用 | 先启动 `servo_controller_node` |
