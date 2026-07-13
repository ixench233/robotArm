# DOFBOT Jetson Nano 机械臂 — SSH 连接指南

## 设备信息

| 项目 | 值 |
|------|-----|
| 设备型号 | Yahboom DOFBOT Jetson Nano 版 |
| 用户名 | `jetson` |
| 密码 | `yahboom` |
| SSH 端口 | `22` |
| JupyterLab 端口 | `8888` |
| IP 地址 | 动态获取（通过 Wi-Fi），当前为 `10.255.177.196` |

---

## 方式一：SSH 命令行连接（推荐）

在终端中执行：

```bash
ssh jetson@<机械臂IP地址>
```

输入密码 `yahboom` 即可登录。

> 如果不知道 IP 地址，可以先看机械臂 OLED 屏幕上显示的 IP，或者登录路由器管理页面查看 DHCP 客户端列表。

### 常用 SSH 登录示例：

```bash
ssh jetson@10.255.177.196
# 密码: yahboom
```

---

## 方式二：JupyterLab 浏览器连接

1. 打开浏览器，访问：`http://<机械臂IP地址>:8888`
2. 输入密码：`yahboom`
3. 进入 JupyterLab 后，可以在 Notebook 中运行 Python 代码控制机械臂，也可以打开 Terminal 执行 Shell 命令。

示例地址：`http://10.255.177.196:8888`

---

## 方式三：通过 VS Code Remote-SSH 连接

1. 安装 VS Code 插件：**Remote - SSH**
2. 按 `F1` → 选择 `Remote-SSH: Connect to Host...`
3. 输入：`jetson@<机械臂IP地址>`
4. 输入密码：`yahboom`
5. 连接后可以直接在 VS Code 中编辑机械臂上的代码文件

---

## 复制粘贴专用提示词（给 AI 助手用）

如果你需要让 AI 助手（如 Claude Code）通过 SSH 帮你检查或操作机械臂，直接把下面这段话发过去：

```text
请通过 SSH 连接到我的 Yahboom DOFBOT Jetson Nano 机械臂：

- 主机：<替换为当前IP>
- 端口：22
- 用户名：jetson
- 密码：yahboom

连接后请先执行只读检查，不要修改、删除任何文件。
如果找不到 IP，可以先尝试 ping 或扫描局域网。
```

---

## 机械臂关键目录速查

| 路径 | 说明 |
|------|------|
| `/home/jetson/Dofbot/` | 官方示例代码主目录 |
| `/home/jetson/Dofbot/0.py_install/Arm_Lib/Arm_Lib.py` | 机械臂控制库源码 |
| `/home/jetson/Dofbot/3.ctrl_Arm/` | 舵机控制基础示例 |
| `/home/jetson/Arm/` | APP/WiFi/系统配置 |
| `/home/jetson/Arm/config.ini` | 颜色识别 + 标定配置 |
| `/home/jetson/dofbot_ws/src/` | ROS 功能包（颜色分拣、垃圾分拣等） |
| `/home/jetson/catkin_ws/src/` | 另一个 ROS 工作空间 |

---

## 快速测试连接是否正常

登录后运行以下命令：

```python
from Arm_Lib import Arm_Device
import time
Arm = Arm_Device()
time.sleep(0.1)
# 蜂鸣器响一声
Arm.Arm_Buzzer_On(1)
# 舵机归中
Arm.Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)
```

如果蜂鸣器响了、舵机动了，说明连接正常。

---

## 注意事项

1. **不要直接修改官方 demo 文件**，需要修改时先复制备份（`cp 原文件 原文件.bak`）
2. **不要删除** `/home/jetson/Dofbot/`、`/home/jetson/Arm/` 下的任何文件
3. **不要执行** `rm`、`reboot`、`shutdown` 命令，除非明确需要
4. **不要修改** `Arm_Lib.py` 底层库代码
5. 任何修改前先做备份

---

## 服务管理命令

```bash
# 查看服务状态
systemctl status jetson_jupyter.service   # JupyterLab
systemctl status dofbot_oled.service      # OLED 屏幕
systemctl status yb-bigProgram.service    # APP 控制

# 查看失败的服务
systemctl --failed

# 查看日志
journalctl -u yb-bigProgram.service -n 50
```

---

> 最后更新：2026-07-01
