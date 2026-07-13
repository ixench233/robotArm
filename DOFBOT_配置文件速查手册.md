# DOFBOT Jetson Nano — 配置文件速查手册

> 本文档记录了机械臂上所有关键配置参数、数据结构和代码约定，供后续写代码时参考。
> 生成日期：2026-07-01

---

## 一、设备身份卡

```text
用户名:   jetson
密码:     yahboom
主机名:   jetson-desktop
系统:     Ubuntu 18.04.6 LTS, aarch64 (Jetson Nano)
内核:     4.9.140-tegra
Python:   3.6.9
ROS:      Melodic
设备版本: Card NO.34, Version 1.0.5 (Released 20221208)
Arm_Lib:  0.0.5 (pip 安装)
```

---

## 二、核心配置文件地图

```
/home/jetson/
├── Arm/
│   ├── config.ini              ★ 主配置文件 (HSV/标定)
│   ├── Arm_WIFI.py             WiFi配网/APP通信
│   ├── YahboomArm.pyc          APP控制程序(编译后,无源码)
│   ├── jupyter.sh              Jupyter启动脚本
│   ├── camera.py               摄像头驱动
│   └── config.ini.bak          配置备份
├── Dofbot/
│   ├── 0.py_install/Arm_Lib/   ★ Arm_Lib 源码
│   │   ├── Arm_Lib.py          核心控制库
│   │   └── setup.py            安装脚本
│   └── ... (教学示例)
├── dofbot_ws/src/              ★ ROS 工作空间
│   ├── dofbot_config/          共享配置包
│   ├── dofbot_info/            运动学/信息服务
│   ├── dofbot_color_sorting/   ★ 颜色分拣
│   ├── dofbot_color_identify/  ★ 颜色识别+抓取
│   ├── dofbot_color_stacking/  ★ 颜色堆叠
│   ├── dofbot_color_follow/    颜色追踪
│   ├── dofbot_garbage_yolov5/  ★ 垃圾分拣(YOLOv5)
│   ├── dofbot_snake_follow/    蛇形跟随
│   ├── dofbot_face_follow/     人脸追踪
│   └── dofbot_moveit/          MoveIt运动规划
├── catkin_ws/src/              旧版ROS工作空间
│   ├── arm_action_group/       ★ 动作组
│   ├── arm_color_grab/         颜色抓取
│   ├── arm_color_identify/     颜色识别
│   ├── arm_color_sorting/      颜色分拣
│   └── ...
└── trace_digits_123.py         ★ 用户自写(画数字123)
```

---

## 三、Arm_Lib 核心库 (`/home/jetson/Dofbot/0.py_install/Arm_Lib/Arm_Lib.py`)

### 初始化
```python
from Arm_Lib import Arm_Device
Arm = Arm_Device()          # I2C 地址 0x15, 总线 smbus.SMBus(1)
```

### 舵机编号与范围

| 舵机 | 名称 | 角度范围 | 库内部处理 |
|------|------|----------|-----------|
| S1 | 底座旋转 | 0-180° | **直接使用**,不反转 |
| S2 | 大臂 | 0-180° | **内部取反** `180 - angle` |
| S3 | 中臂 | 0-180° | **内部取反** `180 - angle` |
| S4 | 小臂 | 0-180° | **内部取反** `180 - angle` |
| S5 | 手腕旋转 | 0-270° | **直接使用**,特殊范围 |
| S6 | 夹爪 | 0-180° | **直接使用**,不反转 |

> **关键理解**：调用 `Arm_serial_servo_write6(90, 90, 90, 90, 90, 90, 500)` 时：
> - S1=90 → 发给硬件 90
> - S2=90 → 发给硬件 **180-90=90** ✓
> - S3=90 → 发给硬件 **180-90=90** ✓
> - S4=90 → 发给硬件 **180-90=90** ✓
> - S5=90 → 发给硬件 90
> - S6=90 → 发给硬件 90
>
> 所以从 API 角度看，所有舵机都是"写 90 就是中间位置"。

### 主要 API

```python
# 单舵机控制
Arm.Arm_serial_servo_write(id, angle, time_ms)
# id: 1-6, angle: 0-180(或0-270 for S5), time_ms: 运行时间

# 6舵机同时控制
Arm.Arm_serial_servo_write6(s1, s2, s3, s4, s5, s6, time_ms)

# 数组方式控制
joints = [s1, s2, s3, s4, s5, s6]
Arm.Arm_serial_servo_write6_array(joints, time_ms)

# 读取舵机角度
angle = Arm.Arm_serial_servo_read(id)  # 返回 0-180

# 蜂鸣器
Arm.Arm_Buzzer_On(delay=0xff)   # 0xff=长响, 1-50=delay*100ms
Arm.Arm_Buzzer_Off()

# RGB灯
Arm.Arm_RGB_set(red, green, blue)  # 0-255

# 扭矩控制
Arm.Arm_serial_set_torque(1)  # 1=锁定, 0=可掰动

# 中位偏移校准
Arm.Arm_serial_servo_write_offset_switch(id)  # id:1-6 设置, 0=恢复默认

# 动作组
Arm.Arm_Action_Mode(0)  # 0=停止, 1=单次, 2=循环
Arm.Arm_Clear_Action()
Arm.Arm_Action_Study()   # 学习模式记录当前动作
```

---

## 四、主配置文件 (`/home/jetson/Arm/config.ini`)

```ini
[calibrateThreshold]
g_calibratethreshold = 115       # 图像二值化阈值

[HSV]
# 格式: H_min, S_min, V_min, H_max, S_max, V_max
g_hsv_red    = 0, 211, 166, 181, 253, 255
g_hsv_green  = 56, 143, 78, 73, 253, 255
g_hsv_blue   = 104, 221, 75, 114, 253, 255
g_hsv_yellow = 16, 190, 188, 27, 253, 255

[calibrateXY]
g_calibratexy = 91, 129          # 标定框识别位置 (S1, S2)
```

### 各 ROS 包的 HSV_config.txt (因光照环境不同,数值略有差异)

| 包 | Red | Green | Blue | Yellow |
|----|-----|-------|------|--------|
| color_sorting | [0,143,145, 8,255,255] | [37,81,48, 78,241,255] | [99,152,160, 118,255,255] | [26,158,149, 35,217,255] |
| color_stacking | [0,143,163, 11,255,255] | [55,113,88, 78,255,255] | [110,169,128, 117,255,255] | [26,100,91, 32,255,255] |
| snake_follow | [0,155,143, 13,255,255] | [54,154,66, 95,255,255] | [99,122,109, 119,255,255] | [10,151,150, 59,255,255] |

---

## 五、标定参数 (`dofbot_config.py` — 7个包完全一致)

```python
class Arm_Calibration:
    def __init__(self):
        self.threshold_num = 130           # 图像二值化阈值
        self.xy = [90, 135]                # ★ 标定框识别时的初始位姿 (S1, S2)
        self.arm = Arm_Lib.Arm_Device()

    def calibration_map(self, image, xy=None, threshold_num=130):
        # 标定时移动到的姿态 (S1, S2, S3, S4, S5, S6)
        joints_init = [self.xy[0], self.xy[1], 0, 0, 90, 30]
        self.arm.Arm_serial_servo_write6_array(joints_init, 1500)
```

> **XYT_config.txt** 格式: `x=90\ny=135\nthresh=169\n`

---

## 六、位姿参数速查表

### 6.1 常用姿态定义 (S1~S5, 不含夹爪S6)

| 姿态名 | S1 | S2 | S3 | S4 | S5 | 用途 |
|--------|----|----|----|----|----|------|
| **归中复位** | 90 | 90 | 90 | 90 | 90 | 所有舵机回90° |
| **p_mould** (归位) | **90** | 130 | 0 | 0 | 90 | ★ 准备抓取/默认归位 |
| **P_HOME** | **90** | 130 | 0 | 0 | 90 | trace_digits_123.py 归位 |
| **p_top** | **90** | 80 | 50 | 50 | 270 | 抬起/架起姿态 |
| **joints_up** | **90** | 80 | 35 | 40 | 90 | color_sorting 架起 |
| **joints_uu** | **90** | 80 | 50 | 50 | 265 | garbage/identify 架起 |
| **joints_00** | **90** | 80 | 50 | 50 | 265 | stacking 架起 |

### 6.2 抓取目标位置 (各颜色积木)

| 颜色 | S1 | S2 | S3 | S4 | S5 | 说明 |
|------|----|----|----|----|----|------|
| **p_Brown** (抓取点) | **90** | 53 | 33 | 36 | 270 | 中间积木堆 |
| **p_Yellow** (放置) | **65** | 22 | 64 | 56 | 270 | 左侧黄区 |
| **p_Red** (放置) | **117** | 19 | 66 | 56 | 270 | 右侧红区 |
| **p_Green** (放置) | **136** | 66 | 20 | 29 | 270 | 右侧绿区 |
| **p_Blue** (放置) | **44** | 66 | 20 | 28 | 270 | 左侧蓝区 |
| **p_gray** (放置) | **90** | 48 | 35 | 30 | 270 | 灰色区域 |

### 6.3 堆叠层位姿 (叠罗汉/搬运工)

| 层 | S1 | S2 | S3 | S4 | S5 |
|----|----|----|----|----|----|
| p_layer_1 (底层) | 90 | 53 | 33 | 36 | 270 |
| p_layer_2 (第二层) | 90 | 63 | 34 | 30 | 270 |
| p_layer_3 (第三层) | 90 | 66 | 43 | 20 | 270 |
| p_layer_4 (第四层) | 90 | 72 | 49 | 13 | 270 |

---

## 七、夹爪 S6 参数

| 场景 | 张开角度 | 夹紧角度 | 动作时间 | 来源 |
|------|----------|----------|----------|------|
| 夹积木 (基础) | **60** | **135** | 400ms | clamp_block.ipynb, action_group.py |
| 颜色分拣 | **30** | **135** | 500ms | color_sorting.py |
| 颜色识别+抓取 | **30** | **135** | 500ms | identify_grap.py |
| 颜色堆叠 | **30** | **140** | 500ms | stacking_grap.py |
| 垃圾分拣 | **30** | **135** | 500ms | garbage_grap.py |
| 画数字123 | **60** | **145** | 450ms | trace_digits_123.py |
| 蛇形跟随 | **30** | **180** | 500ms | snake_ctrl.py |

> **调整夹爪"力度"的方法**：
> - 夹紧角度 ↑ = 夹得更紧 (最大 180)
> - 张开角度 ↓ = 张得更开 (最小 0)
> - 动作时间 ↑ = 给舵机更多时间完成夹合
> - **夹爪角度越大 = 夹爪闭合越紧,因为夹爪是"角度越大小口越小"**

---

## 八、底座 S1 角度与物理方向

所有代码中 S1 的初始值都是 **90°**。

```
S1 = 0   → 底座逆时针转到最左
S1 = 90  → 底座朝向正前方 (默认)
S1 = 180 → 底座顺时针转到最右
```

> **如果你的底座方向反了**：将所有涉及"归位/初始/准备"的 S1=90 改为 S1=0 或 180。
> 建议不要逐个文件改,而是在你自写的代码中加一个全局偏移:
> ```python
> BASE_OFFSET = -90   # 逆时针转90°
> def adj_s1(s1): return (s1 + BASE_OFFSET) % 180
> ```

---

## 九、颜色识别约定

颜色识别的标准流程（以 `color_sorting.py` 为例）：

```python
# 1. 定义 HSV 范围
color_hsv = {
    "red":    ((0, 143, 145), (8, 255, 255)),
    "green":  ((37, 81, 48), (78, 241, 255)),
    "blue":   ((99, 152, 160), (118, 255, 255)),
    "yellow": ((26, 158, 149), (35, 217, 255)),
}

# 2. 抓取流程
self.grap_joint = 135  # S6 夹紧角度
joints_up = [90, 80, 35, 40, 90, self.grap_joint]  # 架起
self.joints = [90, 53, 33, 36, 90, 30]               # 抓取位置

# 3. 移动流程 (sorting_move):
#    架起 → 松夹爪(30) → 下降抓取 → 夹紧(135) → 抬起 → 旋转底座 → 放置 → 松开(30) → 返回归位
```

---

## 十、服务管理

| 服务名 | 状态 | 启动命令 |
|--------|------|----------|
| `jetson_jupyter.service` | **active** | `bash /home/jetson/Arm/jupyter.sh` (JupyterLab :8888) |
| `dofbot_oled.service` | **active** | `python3 /home/jetson/Dofbot/2.sys_settings/1.OLED/oled.py` |
| `yb-bigProgram.service` | **failed** | `python3 /home/jetson/Arm/YahboomArm.pyc` (APP控制) |

### 手动启动 APP 服务
```bash
sudo systemctl start yb-bigProgram.service
# 或手动运行
cd /home/jetson/Arm && python3 YahboomArm.pyc
```

---

## 十一、硬件信息

```text
I2C 总线: /dev/i2c-0 ~ /dev/i2c-8
Arm_Lib 使用: I2C-1 (smbus.SMBus(1))
舵机控制板地址: 0x15
摄像头: /dev/video0 (USB摄像头)
OLED: SSD1306 128x32, I2C-1
GPIO: Jetson.GPIO 2.0.11
```

---

## 十二、网络配置

```text
当前 WiFi:  GGrobot (通过 nmcli 连接)
备用 WiFi:  Yahboom (已保存配置)
WiFi 配网:  /home/jetson/Arm/Arm_WIFI.py (二维码扫描配网)
有线网口:   eth0 (未连接)
Docker 网桥: docker0 (172.17.0.1/16)
```

### .bashrc 关键环境变量
```bash
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda/bin:$PATH
source /opt/ros/melodic/setup.bash
source /home/jetson/catkin_ws/devel/setup.bash
source /home/jetson/dofbot_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

---

## 十三、写代码时的最佳实践

### 推荐的导入模板
```python
#!/usr/bin/env python3
# coding: utf-8
import time
from Arm_Lib import Arm_Device

# 创建机械臂实例
arm = Arm_Device()
time.sleep(0.1)

# === 你的可调参数 ===
BASE_OFFSET = 0      # S1 底座偏移 (如需逆时针转90°: -90)
GRIP_CLOSED = 135    # S6 夹紧角度 (想更紧: 140-160)
GRIP_OPEN = 30       # S6 张开角度
MOVE_TIME = 700      # 默认移动时间 ms

# === 姿态定义 ===
HOME  = [90, 130, 0, 0, 90]      # 归位 (S1~S5)
RAISE = [90, 80, 50, 50, 270]    # 抬起
# 注意: HOME 和 RAISE 都是5元素(不含S6),需要补S6才能调用
```

### 安全守则
1. **不要直接改官方文件**, 先 `cp 原文件 原文件.bak`
2. **Arm_Lib.py 不要动** — 它是全局依赖
3. **config.ini 不要直接改** — 被多个程序共享
4. 在你的代码中做参数覆盖,而不是修改官方代码
5. 测试新角度时先用低速 (time=1000+), 确认没问题再加速

---

> 提示：在 JupyterLab (`http://<IP>:8888`) 中可以直接运行 Notebook 测试,也可以打开 Terminal 执行 Python 脚本。
