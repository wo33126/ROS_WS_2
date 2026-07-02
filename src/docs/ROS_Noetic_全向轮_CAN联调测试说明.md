# ROS Noetic 全向轮 + STM32(CAN) 联调测试说明

本文档用于验证以下链路：

1. `/cmd_vel` -> `motion_planner` 全向轮解算
2. `/motor_velocity_cmd` -> `can_motor_interface` 自定义 CAN 速度帧
3. STM32 收到后转发给驱动器，并回传状态帧

---

## 1. 当前协议与节点约定

### 1.1 ROS -> STM32 速度控制帧

- CAN ID: `0x201`（默认，扩展帧）
- DLC: `7`
- Payload:
  - Byte0: 命令码 `0x01`（速度控制）
  - Byte1: 电机索引 `0~3` 或 `0xFF`（广播）
  - Byte2~5: `int32_t` 目标转速（RPM，默认小端）
  - Byte6: 校验（默认 `sum8`，即前6字节求和低8位）

### 1.2 STM32 -> ROS 状态帧

- CAN ID: `0x181`（默认，扩展帧）
- Byte0: `0x81`（状态上报）
- Byte1: 电机索引
- Byte2: 状态标志（可选）
- Byte3~6: `int32_t` 实际转速 RPM（可选）
- 最后1字节: 校验（sum8）

ROS 侧发布：
- `/motor_state` (`std_msgs/Float32MultiArray`)
- `/motor_status_flags` (`std_msgs/UInt8MultiArray`)

---

## 2. 启动步骤

先配置 CAN（示例 1Mbps）：

```bash
sudo ip link set can0 down || true
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

编译并加载环境：

```bash
cd ~/ros_ws
catkin_make -j4 -l4
source devel/setup.bash
```

启动整机（不启用键盘，便于复现）：

```bash
roslaunch robot_bringup robot_bringup.launch enable_teleop:=false
```

另开一个终端抓包：

```bash
candump can0
```

---

## 3. 用 rostopic 手动发送 Twist

### 3.1 前进 0.5 m/s（`vy=0, wz=0`）

```bash
rostopic pub /cmd_vel geometry_msgs/Twist \
"linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" -r 10
```

本项目默认参数：`wheel_radius=0.05m`，`wheel_base=0.18m`。

全向轮解算（前/右/后/左）：

- `v1 = vx + ωL = 0.5`
- `v2 = vy + ωL = 0.0`
- `v3 = -vx + ωL = -0.5`
- `v4 = -vy + ωL = 0.0`

RPM 计算：`RPM = (v * 60) / (2πr)`

- 前轮约 `+95.5 RPM`（发送时取整为 `95`）
- 右轮 `0 RPM`
- 后轮约 `-95.5 RPM`（取整 `-95`）
- 左轮 `0 RPM`

### 3.2 预期 candump 现象（示例）

应持续看到 4 条发送帧（索引 0/1/2/3）：

- idx0, +95RPM: 数据近似 `01 00 5F 00 00 00 60`
- idx1, 0RPM: 数据近似 `01 01 00 00 00 00 02`
- idx2, -95RPM: 数据近似 `01 02 A1 FF FF FF A1`
- idx3, 0RPM: 数据近似 `01 03 00 00 00 00 04`

> 说明：校验为前6字节求和低8位；如果你将 `payload_little_endian` 或 `checksum_use_sum8` 改掉，抓包字节会不同。

---

## 4. 观察 STM32 回传与 ROS 状态

查看 ROS 状态：

```bash
rostopic echo /motor_state
rostopic echo /motor_status_flags
```

正常情况下：

1. `candump` 中有 `0x201` 下发帧；
2. `candump` 中有 `0x181` 上报帧；
3. `/motor_state` 数据跟随命令变化；
4. 负转速应能正确显示为负值。

---

## 5. 常见问题排查

1. 看不到 `0x201`：
   - 检查 `can_interface_node` 是否启动。
   - 检查是否有 `/motor_velocity_cmd` 输入。

2. 有 `0x201` 但没有 `0x181`：
   - 检查 STM32 是否已实现状态上报命令 `0x81`。
   - 检查 `rx_can_id` 是否与 STM32 实际发送 ID 一致。

3. 有上报但 ROS 不更新：
   - 校验方式不一致（sum8/XOR）。
   - 大小端不一致（`payload_little_endian`）。
   - 电机索引越界（应在 `0~3` 或 `0xFF`）。

4. 驱动器转速异常：
   - 确认 ROS 下发单位是 RPM（整数）。
   - X 固件 `RPM*10` 换算由 STM32 执行，ROS 不再乘10。
