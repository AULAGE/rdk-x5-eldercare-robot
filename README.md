# 智慧康养双机协同机器人 · 嵌入式代码

基于多模态光电感知与 AI Agent 的双机协同智慧康养系统的**嵌入式侧源码**。系统由六足巡检机器人、升降机械臂大车与自研"颐伴"多模态光电感知板组成，决策层运行于地平线 RDK X5 边缘计算平台，各节点经 ROS 2 协同通信。

本仓库收录系统中各 MCU 固件工程与六足平台的 ROS 2 工作空间。

## 仓库结构

```
├── sensor_v2.1/          # "颐伴"多模态光电感知板固件（STM32F103C8T6，Keil MDK）
├── PWM机械臂/            # 6 自由度机械臂 PWM 舵机控制（STM32F10x，Keil MDK）
├── 伺服电机控制/          # 丝杠升降 Z 轴步进电机脉冲控制（STM32F103C8T6，Keil MDK）
├── 六足机器人代码/        # 六足平台 ROS 2 工作空间（树莓派 4B 侧）
└── 大车代码/              # 机械臂大车底盘固件（STM32F407VET6 + FreeRTOS）
```

## 模块说明

### sensor_v2.1 —— 光电感知板固件

自研"颐伴"光电感知板的采集与告警固件：

- 四类传感信号采集：DHT11 温湿度（单总线）、红外火焰、MQ-2 烟雾、光敏光照（火焰/烟雾/光照均为 AO 模拟量 + DO 数字量双通道）
- AO / DO 双判定模式，按键切换：DO 越限速判、AO 阈值精判
- 越限直驱蜂鸣器本地告警（不依赖网络）
- 串口周期上报环境数据
- `APP/` 下按外设分模块（adc、dht11、MQ-2、sensor、beep、usart 等），另含红外、超声波、舵机等可插拔扩展驱动

### PWM机械臂 —— 机械臂舵机控制

大车上层 6 自由度机械臂的 PWM 舵机控制工程（OpenArmSTM32），负责关节角度控制与夹爪抓放。

### 伺服电机控制 —— 升降平台步进控制

丝杠升降 Z 轴的步进电机脉冲控制，提供串口 ASCII 指令协议（定位 / 急停 / 归零 / 位置查询），带限位截断与运动互斥保护。**指令协议与驱动层 API 详见目录内 [readme.md](伺服电机控制/readme.md)。**

### 六足机器人代码 —— ROS 2 工作空间

六足平台（树莓派 4B）的建图导航与传感器驱动栈，基于轮趣科技开源驱动包配置与二次开发：

| 包 | 功能 |
|---|---|
| `lslidar_driver` | 镭神激光雷达驱动 |
| `hipnuc_imu` / `imu_20498` | IMU 驱动 |
| `astra_camera(_msgs)` | 奥比中光深度相机驱动（含 OpenNI2 运行库） |
| `wheeltec_cartographer` | Cartographer 2D SLAM 建图 |
| `wheeltec_nav2` | Nav2 导航配置 |
| `wheeltec_path_follow` | 路径跟随 |
| `stm32_to_chassis` | 上位机 ↔ STM32 底盘桥接 |
| `wheeltec_robot_msg` | 自定义消息 |

### 大车代码 —— 大车底盘固件

机械臂大车轮式底盘固件（STM32F407VET6 + FreeRTOS），霍尔编码器速度闭环、ICM20948 姿态解算、串口速度指令接口，基于轮趣科技开源底盘例程二次开发。

## 编译与运行

**STM32 固件**（sensor_v2.1 / PWM机械臂 / 伺服电机控制 / 大车代码）：

1. 安装 Keil MDK5 及对应器件包（STM32F1xx / STM32F4xx DFP）
2. 打开各目录下的 `.uvprojx` / `.uvproj` 工程编译
3. 经 ST-Link / J-Link 烧录

**ROS 2 工作空间**（六足机器人代码）：

```bash
# ROS 2 环境（树莓派 4B / Ubuntu）
cd 六足机器人代码
colcon build
source install/setup.bash
# 依次启动雷达、IMU、底盘桥接与建图/导航 launch
```

## 说明

- 决策层 AI Agent（RDK X5 上基于开源 OpenClaw 框架二次开发的康养 Agent、MCP 工具集与 ADB 控制链路）不在本仓库内。
- 仓库中包含的第三方组件（STM32 标准外设库、FreeRTOS、轮趣/镭神/HiPNUC/奥比中光驱动等）版权归原作者所有，遵循其各自开源许可。
