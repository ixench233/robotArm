# 机械臂项目

## 功能 / 创新点

- 多模态自然交互与 Web 可视化控制：系统支持文本聊天、自然语言指令、网页按钮、视频画面和手势等多种输入方式，并在网页端集成摄像头画面、舵机控制、动作组和状态展示，将不同表达方式统一映射为机械臂动作意图，降低调试和演示门槛。
- 视觉感知驱动的智能任务执行：系统结合机械臂摄像头、人脸追踪、位置预瞄、棋盘与棋子视觉识别等能力，可根据自然语言任务描述和视觉识别结果完成动作决策，例如人脸跟随、九宫格三子棋落子判断和人机对弈。
- 自定义动作学习与统一调度：用户可以通过手势触发动作、绑定自定义动作组，也可以手动设置多个机械臂姿态并保存为动作序列；系统通过统一动作调度管理舵机级精细控制、动作回放和多功能协同执行。

## 视频中提取的演示亮点

- 聊天功能：展示“不同表达方式下的相同意图”，说明系统不是只依赖固定按钮，而是具备一定语义理解能力。
- 视频互动：展示网页端同时支持机械臂摄像头画面，并可直接进行六轴机械臂角度控制。
- 手势互动：展示通过手势控制机械臂，并支持绑定自定义动作。
- 人脸追踪：展示机械臂根据人的位置进行预瞄。
- 记忆学习：展示用户手动设置多个机械臂姿态并保存、复现。
- 三子棋任务：展示棋盘视觉识别、落子判断和人机对弈过程。

## 功能分工边界

已由其他成员负责：

- 九宫格三子棋挑战：包含棋盘识别、落子策略、棋子抓取和人机对弈流程。
- 舵机级精细控制：包含单舵机角度调试、六轴姿态细调、夹爪角度和底层安全动作。

当前可重点负责：

- Web 可视化控制平台
- 摄像头实时视频流
- 聊天 / 自然语言意图控制
- 手势识别触发动作组
- 人脸追踪与位置预瞄
- 记忆学习模式的动作记录和回放管理
- 各功能的统一任务调度和演示界面

## 具体实现方法

### 1. Web 可视化控制平台

实现目标：

- 在浏览器中统一展示摄像头画面、聊天输入、功能按钮、当前模式和运行状态。
- 页面按钮不直接写舵机角度，而是发送高层动作命令，例如 `wave`、`nod`、`face_follow_on`、`gesture_on`。

实现方案：

- 后端使用 `Flask`，运行在 Jetson Nano 上。
- 摄像头流使用 OpenCV 读取 `/dev/video0`，通过 MJPEG 接口输出到网页。
- 前端使用普通 HTML + JavaScript，调用后端接口：
  - `GET /video_feed`：摄像头实时画面
  - `POST /api/chat`：提交自然语言指令
  - `POST /api/action`：触发动作组
  - `POST /api/mode`：切换模式，如手势识别、人脸追踪、普通监控
  - `GET /api/status`：获取当前状态

可复用资料 / 源码：

- `21.程序源码汇总/新源码/Arm/camera.py`
- `21.程序源码汇总/新源码/Arm/templates/index.html`
- `21.程序源码汇总/新源码/Arm/open_camera.py`

### 2. 摄像头实时视频流

实现目标：

- 网页端能实时看到机械臂摄像头画面。
- 后续人脸追踪、手势识别都复用同一个摄像头采集模块。

实现方案：

- 使用 `cv2.VideoCapture(0)` 读取摄像头。
- 设置分辨率为 `640x480`，编码为 MJPG。
- 后端将每帧编码为 JPG，再按 MJPEG 流返回浏览器。
- 摄像头读取模块只创建一个全局实例，避免多个功能同时抢占摄像头。

关键代码来源：

- `VideoCamera.get_frame()`
- `VideoCamera.get_frame2()`

风险点：

- 如果摄像头打不开，先检查 `/dev/video0` 是否存在。
- 如果官方 APP 或其他程序占用摄像头，需要停止对应服务或进程。

### 3. 聊天 / 自然语言意图控制

实现目标：

- 用户输入自然语言，系统识别意图并触发对应功能。
- 第一版不依赖大模型，先用关键词规则保证稳定演示。

实现方案：

- 建立一个意图映射表：

```python
INTENTS = {
    "挥手": "wave",
    "打招呼": "wave",
    "点头": "nod",
    "鼓掌": "applaud",
    "跳舞": "dance",
    "看着我": "face_follow_on",
    "人脸追踪": "face_follow_on",
    "停止": "stop",
    "手势控制": "gesture_on",
}
```

- `/api/chat` 接收文本后，先做关键词匹配。
- 匹配成功后调用统一动作调度器。
- 匹配失败时返回提示，例如“未识别指令，请换一种说法”。
- 后续如果时间充足，再把规则识别替换为大模型 API，但外部接口保持不变。

实现重点：

- 自然语言模块只输出动作名，不直接控制舵机。
- 动作执行交给 `ActionManager`，避免和其他成员的舵机控制逻辑冲突。

### 4. 手势识别触发动作组

实现目标：

- 摄像头识别手势后触发机械臂动作。
- 不负责底层角度细调，只负责“手势 -> 动作名”的映射。

推荐方案：

- 优先使用 Mediapipe 本地识别，不使用百度云手势 API。
- 识别手部 21 个关键点后，判断手指伸展状态。
- 将手势映射到动作：
  - 五指张开：鼓掌
  - OK：挥手
  - 食指单独伸出：点头
  - 剪刀手：跳舞
  - 拇指向下：停止 / 复位

可复用资料 / 源码：

- `18.Mediapipe开发/1、手部检测`
- `18.Mediapipe开发/10、手势识别`
- `18.Mediapipe开发/11.Mediapipe手势控制机械臂动作组`
- `21.程序源码汇总/新源码/dofbot_ws/src/arm_mediapipe/scripts/media_library.py`
- `21.程序源码汇总/新源码/dofbot_ws/src/arm_mediapipe/scripts/FingerCtrl.py`
- `21.程序源码汇总/新源码/dofbot_ws/src/arm_mediapipe/scripts/ArmCtrl.py`

实现重点：

- 手势识别线程只更新 `current_gesture` 和触发动作。
- 加入冷却时间，例如 2 秒内同一手势只触发一次，避免机械臂重复执行。
- 页面显示当前识别结果，方便答辩演示。

### 5. 人脸追踪与位置预瞄

实现目标：

- 检测画面中最大人脸。
- 根据人脸中心点相对画面中心的偏移，驱动机械臂朝向人脸方向。

实现方案：

- 第一版直接复用 OpenCV Haar 人脸检测，比 Mediapipe FaceMesh 更轻。
- 画面中心设为 `(320, 240)`。
- 检测到人脸后计算：

```python
offset_x = face_center_x - 320
offset_y = face_center_y - 240
```

- 使用 PID 或比例控制将偏移转为头部预瞄动作。
- 如果底层舵机控制由其他同学负责，这里只输出偏移量和目标方向：

```python
{
    "mode": "face_follow",
    "offset_x": -80,
    "offset_y": 35
}
```

可复用资料 / 源码：

- `13.AI视觉追踪/4.人脸定位实验`
- `13.AI视觉追踪/5.人脸追踪实验`
- `21.程序源码汇总/新源码/dofbot_ws/src/dofbot_color_follow/face_follow.py`
- `21.程序源码汇总/新源码/dofbot_ws/src/dofbot_color_follow/PID.py`
- `21.程序源码汇总/新源码/dofbot_ws/src/dofbot_color_follow/haarcascade_frontalface_default.xml`

实现重点：

- 不要一开始就追求机械臂动作很准，先完成“框出人脸 + 输出偏移 + 页面显示状态”。
- 动作执行时限制最大变化量，避免机械臂快速抖动。

### 6. 记忆学习模式

实现目标：

- 用户能记录多个动作步骤，保存为动作序列。
- 后续一键回放，完成视频中“记忆学习”的演示效果。

实现方案：

- 页面提供按钮：
  - 记录当前姿态
  - 添加动作
  - 删除动作
  - 保存动作组
  - 回放动作组
- 动作序列保存为 JSON 文件，例如 `actions/hello.json`：

```json
[
  {"name": "home", "joints": [90, 135, 0, 45, 90, 90], "duration": 1000},
  {"name": "wave_left", "joints": [60, 120, 30, 45, 90, 90], "duration": 800},
  {"name": "wave_right", "joints": [120, 120, 30, 45, 90, 90], "duration": 800}
]
```

- 如果不能直接读取当前舵机角度，可以先让负责舵机的同学提供“读取当前姿态”的接口。
- 如果暂时没有读取接口，也可以先做“预设动作组保存和回放”，满足演示需求。

可复用资料 / 源码：

- `21.程序源码汇总/新源码/Dofbot/3.ctrl_Arm/8.study_mode.ipynb`
- `DOFBOT_配置文件速查手册.md` 中 `Arm_Action_Study()`、`Arm_Clear_Action()`、`Arm_Action_Mode()`

实现重点：

- 记忆学习模块负责动作序列的数据管理。
- 底层动作执行统一交给 `ActionManager`，避免重复写机械臂控制代码。

### 7. 统一动作调度器

实现目标：

- 聊天、按钮、手势、人脸追踪都不要各自直接控制机械臂。
- 所有模块统一提交动作命令，减少冲突。

实现方案：

- 新建 `ActionManager`：

```python
class ActionManager:
    def __init__(self):
        self.current_mode = "idle"
        self.running = False

    def run_action(self, action_name, source="web"):
        if self.running:
            return {"ok": False, "msg": "机械臂正在执行动作"}
        # 根据 action_name 调用对应动作
        return {"ok": True, "action": action_name, "source": source}

    def stop(self):
        self.current_mode = "idle"
        self.running = False
```

- 各模块只调用：

```python
action_manager.run_action("wave", source="gesture")
action_manager.run_action("face_follow_on", source="chat")
```

实现重点：

- 加锁防止多个功能同时控制机械臂。
- 所有动作记录来源，方便页面展示“当前由手势/聊天/按钮触发”。

### 8. 与其他成员接口约定

为了不和三子棋、舵机控制同学冲突，建议约定一个最小接口：

```python
run_action(action_name: str) -> dict
stop_action() -> dict
get_status() -> dict
get_current_joints() -> list
move_to_joints(joints: list, duration: int) -> dict
```

当前负责内容只调用这些接口，不直接修改三子棋代码和底层舵机库。

## 推荐开发顺序

1. 跑通 SSH 环境和摄像头读取。
2. 搭建 Flask Web 页面，显示实时视频流。
3. 实现 `/api/action`，先用预设动作模拟调用。
4. 实现聊天关键词意图识别。
5. 接入 Mediapipe 手势识别，并触发动作组。
6. 接入 OpenCV 人脸检测，先显示人脸框和偏移量。
7. 将人脸偏移交给动作调度器或舵机控制同学接口。
8. 实现记忆学习 JSON 保存和回放。
9. 最后统一页面状态展示和答辩演示流程。

## 当前实现状态

已实现代码目录：

- `web_control/`

已实现功能：

- Web 控制台页面：`web_control/templates/index.html`
- 页面样式和交互：`web_control/static/styles.css`、`web_control/static/app.js`
- Flask 后端接口：`web_control/app.py`
- 机械臂真机 / 模拟适配：`web_control/robot_arm.py`
- 摄像头、可选人脸检测、可选手势识别：`web_control/vision.py`
- 记忆动作组保存和回放：`web_control/action_store.py`、`web_control/actions/demo.json`

运行方式：

```bash
cd web_control
python -m pip install -r requirements.txt
python app.py --host 0.0.0.0 --port 5000
```

浏览器访问：

```text
http://<Jetson-IP>:5000
```

本机测试地址：

```text
http://127.0.0.1:5000
```

接口约定：

- `GET /video_feed`：摄像头实时画面
- `GET /api/status`：系统状态
- `POST /api/chat`：自然语言指令
- `POST /api/action`：执行动作
- `POST /api/mode`：切换模式
- `GET /api/joints/current`：读取当前姿态
- `POST /api/joints/move`：移动到指定姿态
- `GET /api/sequences`：动作组列表
- `POST /api/sequences/<name>/record`：记录当前姿态
- `POST /api/sequences/<name>/play`：回放动作组

实现说明：

- 如果 Jetson 上存在 `Arm_Lib`，程序会调用真实机械臂。
- 如果当前环境没有 `Arm_Lib`，程序自动进入模拟模式，方便本机开发和答辩前调试页面。
- 手势识别依赖 `mediapipe`，未安装时页面仍能运行，只是手势识别显示为可选不可用。
- 人脸追踪依赖 OpenCV 的 Haar 分类器，若当前 OpenCV 包不支持分类器，则自动降级，不影响其它功能。

## 答辩分工

| 任务 | 负责人 | 备注 |
| --- | --- | --- |
| 拍照 / 录视频 | 待定 | 展示机械臂功能效果 |
| 制作 PPT | 待定 | 汇总项目背景、功能、创新点和成果 |
| 撰写报告 | 待定 | 整理设计思路、实现过程和测试结果 |
| 剪辑视频 | 待定 | 用于答辩展示和项目演示 |

## 网站
https://www.yahboom.com/study/Dofbot-Jetson_nano
