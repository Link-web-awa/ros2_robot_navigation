# ros2_robot_navigation

这是一个基于 ROS 2 的机器人底盘控制与四路传感器数据接入项目，主要用于移动底盘、电机/IMU 反馈、超声波/激光雷达等模块数据读取与 ROS 话题发布。

## 项目概览

该工作区包含两部分：

- `test_ws/src`：实际可编译的 ROS 2 包，包含底盘节点、四路传感器节点、自定义消息定义和串口依赖。
- `四路传感器模块节点代码/`：四路传感器模块相关节点源码备份/参考实现。

当前核心功能包括：

- 通过串口与底盘控制器通信
- 接收 `cmd_vel` 指令，并转换为底盘控制帧
- 读取底盘反馈的线速度、角速度、IMU 原始数据和电池电压
- 接收四路传感器模块数据（超声波、STP23、LD14P）
- 发布对应 ROS 话题，供导航或上层应用使用

## 目录结构

```text
ros2_robot_navigation/
├── README.md
├── 四路传感器模块节点代码/
│   └── s21c_receive_data/
│       ├── src/
│       ├── launch/
│       ├── CMakeLists.txt
│       └── package.xml
└── test_ws/
    └── src/
        ├── chassis/
        │   ├── launch/
        │   ├── src/
        │   ├── CMakeLists.txt
        │   └── package.xml
        ├── s21c_receive_data/
        │   ├── launch/
        │   ├── src/
        │   ├── CMakeLists.txt
        │   └── package.xml
        ├── s21c_receive_msg/
        │   ├── msg/
        │   ├── CMakeLists.txt
        │   └── package.xml
        └── serial/
            ├── include/
            ├── src/
            ├── examples/
            └── README.md
```

## ROS 2 包说明

### 1. `chassis`

作用：底盘串口通信节点，负责与 WHEELTEC STM32 机器人底盘控制器通信。

关键行为：

- 订阅 `/cmd_vel`
- 发送控制帧到串口设备
- 从串口读取底盘反馈帧
- 发布：
  - `/car/actual_velocity`（`geometry_msgs/msg/Twist`）
  - `/car/imu_raw`（`std_msgs/msg/Int16MultiArray`）
  - `/car/battery_voltage`（`std_msgs/msg/Float32`）

对应代码：

- `test_ws/src/chassis/src/chassis_communication.cpp`
- `test_ws/src/chassis/launch/chassis_bringup.launch.py`

### 2. `s21c_receive_data`

作用：四路传感器模块串口数据读取节点，支持不同传感器协议。

该节点通过 `module_n` 参数切换模式：

- `0`：超声波模块
- `1`：STP23 模块
- `2`：LD14P 激光雷达模块

支持发布：

- `/distance`（`s21c_receive_msg/msg/Distance`）
- `/STP23`（`s21c_receive_msg/msg/STP23`）
- `/lidar_data`（`s21c_receive_msg/msg/Lidar`）

对应代码：

- `test_ws/src/s21c_receive_data/src/s21c_listener.cpp`

### 3. `s21c_receive_msg`

作用：自定义 ROS 消息定义。

定义消息：

- `Distance.msg`
- `STP23.msg`
- `Lidar.msg`

### 4. `serial`

作用：串口通信依赖库，供传感器读取节点使用。

来源：

- `test_ws/src/serial`

## 开发环境

- Ubuntu 22.04
- ROS 2 Humble
- CMake 3.8+
- C++17

## 编译方式

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/bl/ros2_robot_navigation/test_ws
colcon build --symlink-install
source install/setup.bash
```

如果需要清理重编译：

```bash
cd /home/bl/ros2_robot_navigation/test_ws
rm -rf build install log
colcon build --symlink-install
```

## 启动方式

项目提供了统一的 launch 文件：

```bash
source /home/bl/ros2_robot_navigation/test_ws/install/setup.bash
ros2 launch chassis chassis_bringup.launch.py
```

该 launch 文件会同时启动：

- 底盘控制节点：`chassis_communication`
- 传感器节点：`s21c_listener`

参数说明：

- `chassis_device`：底盘控制器串口设备，默认 `/dev/serial/by-id/...`
- `chassis_baud_rate`：底盘控制器波特率，默认 `115200`
- `s21c_port`：四路传感器串口设备，默认 `/dev/serial/by-id/...`
- `s21c_module`：传感器模式，默认 `1`

也可以手动覆盖参数：

```bash
ros2 launch chassis chassis_bringup.launch.py \
  chassis_device:=/dev/ttyACM1 \
  chassis_baud_rate:=115200 \
  s21c_port:=/dev/ttyACM0 \
  s21c_module:=2
```

## 常用 ROS 话题

启动后，可以查看话题：

```bash
ros2 topic list
```

常见主题：

- `/cmd_vel`
- `/car/actual_velocity`
- `/car/imu_raw`
- `/car/battery_voltage`
- `/distance`
- `/STP23`
- `/lidar_data`

例如查看底盘速度：

```bash
ros2 topic echo /car/actual_velocity
```

## 注意事项

- 串口设备路径需要根据实际硬件进行校正，常见为 `/dev/ttyUSB*`、`/dev/ttyACM*` 或 `/dev/serial/by-id/*`。
- `LD14P` 模块使用较高波特率（代码中默认 `921600`）。
- `module_n` 参数必须与实际硬件模块一致，否则数据解析可能失败。
- 若串口未正确打开，节点会输出错误日志并退出。

## 典型使用流程

1. 确认底盘控制器和传感器模块对应串口设备。
2. 更新 launch 参数中的设备路径。
3. 编译 ROS 工作空间。
4. 启动 `chassis_bringup.launch.py`。
5. 通过 ROS 话题接收和处理底盘/传感器数据。
6. 如需导航或避障，可再接入 `nav2`、SLAM 或上层逻辑节点。

## 说明

当前仓库主要偏向底盘通信和传感器接入层，适合进一步扩展为导航、建图和避障闭环控制。若后续需要接入 SLAM、路径规划或机器人控制算法，可以在此基础上继续开发。
