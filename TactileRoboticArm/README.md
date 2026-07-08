# 基于 Intel RealSense、OpenCV 与 MediaPipe 的体感机械臂

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.11+-blue.svg" alt="Python Version">
  <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20macOS%20%7C%20Windows-lightgrey.svg" alt="Platform Support">
  <img src="https://img.shields.io/badge/License-GPL--3.0-blue.svg" alt="License">
  <a href="https://github.com/astral-sh/uv"><img src="https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/uv/main/assets/badge/v0.json" alt="uv"></a>
</p>

---

## ✨ 核心特性

- 🛡️ **高精度手势解算**：深度集成 OpenCV 图像处理与 MediaPipe Hands 解决方案，结合 RealSense 深度信息，获取手部 21 个 3D 关键点。
- ⚡ **低延迟通信**：基于异步 WebSockets 的上位机与下位机通信，确保操控的实时性。
- 🧩 **稳健的逆运动学 (IK)**：利用 `ikpy` 库和自定义 URDF 模型，实现平滑的坐标到角度转换。
- 📉 **多级滤波处理**：内置指数平滑滤波器（EMA），有效滤除视觉追踪噪声，防止机械臂剧烈抖动。
- 📏 **工作空间映射**：支持将摄像头视场（FOV）动态映射至机械臂可达空间。
- 📦 **现代包管理**：使用 `uv` 管理 Python 环境，确保依赖一致性与快速安装。
- 🖼️ **实时视觉反馈**：通过 **OpenCV** 实现低延迟的画面渲染、手部关键点标注及空间坐标实时显示。

---

## 🏗️ 系统架构

本系统采用 **C/S (Client/Server)** 架构设计：

1.  **上位机 (PC - Client)**:
    *   **视觉模块**：获取 RealSense 深度流，运行 MediaPipe 进行手部关键点提取。
    *   **处理模块**：执行坐标转换、工作空间映射及滤波。
    *   **运动学模块**：根据 3D 目标点，通过 IK 求解各关节舵机角度。
    *   **通信模块**：将角度指令打包并通过 WebSocket 发送至下位机。
2.  **下位机 (Jetson Nano - Server)**:
    *   **中转服务**：运行基于 `websockets` 的异步服务器。
    *   **驱动模块**：调用 `Arm_Lib` 硬件库，驱动 PWM 舵机执行动作。

---

## 🛠️ 硬件需求

| 组件 | 推荐型号 | 作用 |
| :--- | :--- | :--- |
| **深度相机** | Intel RealSense D456 / D455 / D435 | 获取 3D 手势信息 |
| **机械臂** | Dofbot (Jetson Nano 版) | 6 自由度执行机构 |
| **核心板** | Jetson Nano (4GB) | 运行下位机驱动程序 |
| **控制端** | 笔记本或台式 PC (Win/Linux/macOS) | 运行视觉与 IK 算法 |

---

## 🚀 快速开始

### 1. 克隆项目

```bash
git clone https://github.com/your-username/TactileRoboticArm.git
cd TactileRoboticArm
```

### 2. 下位机配置 (Jetson Nano)

确保 Jetson Nano 已连接网络并安装了官方 `Arm_Lib` 驱动。

```bash
cd jetson_nano_server
# 安装依赖
pip3 install websockets
# 运行服务
python3 main.py
```
*记下终端输出的 IP 地址，例如 `ws://192.168.1.100:8765`。*

### 3. 上位机配置 (PC)

推荐使用 [uv](https://github.com/astral-sh/uv) 管理环境。

```bash
cd hand_tracker
# 安装依赖并同步环境
uv sync
```

### 4. 运行控制程序

在 `hand_tracker/robot_control/config.py` 中修改 `JETSON_IP` 为你下位机的 IP。

```bash
# 启动体感追踪
uv run main.py
```

---

## ⚙️ 关键配置说明

在 `hand_tracker/robot_control/config.py` 中，你可以调整以下参数以优化体验：

*   `TRANSMISSION_INTERVAL`: 发送指令的时间间隔（默认 0.05s）。
*   `SMOOTHING_FACTOR`: 坐标平滑系数（0-1），值越小越平滑，但延迟越高。
*   `WORKSPACE_LIMITS`: 机械臂在 X, Y, Z 方向的可达范围。
*   `SERVO_K / SERVO_B`: 舵机校准斜率与截距。

---

## 📂 项目结构

```text
.
├── hand_tracker/           # 上位机程序 (PC)
│   ├── robot_control/      # 核心逻辑 (IK、通信、视觉)
│   ├── urdf/               # Dofbot 机械臂模型
│   ├── main.py             # 主入口 (Hand Tracking Loop)
│   └── workspace.py        # 工作空间可视化与计算工具
├── jetson_nano_server/     # 下位机程序 (Jetson Nano)
│   └── main.py             # WebSocket Server & 硬件驱动
├── GEMINI.md               # 开发者上下文说明
└── README.md               # 项目主文档
```
---

## 🤝 贡献指南

1. Fork 本仓库
2. 创建你的特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交你的修改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启一个 Pull Request

---

## 📄 许可证

本项目采用 [GNU General Public License v3.0 (GPL-3.0)](LICENSE) 开源。

---

## 🙏 鸣谢

- [MediaPipe](https://github.com/google-ai-edge/mediapipe)
- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [IKPy](https://github.com/Phylliade/ikpy)
- [Yahboom Dofbot](https://github.com/YahboomTechnology/dofbot-jetson_nano)
