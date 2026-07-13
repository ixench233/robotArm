# DOFBOT Web Control

This directory contains a runnable Web control prototype for the DOFBOT Jetson Nano arm.

Implemented features:

- Web dashboard
- Camera MJPEG stream
- Chat / natural language intent matching
- Action buttons
- Optional face tracking
- Optional Mediapipe gesture recognition
- Action sequence save / replay
- Simulation mode when `Arm_Lib` is unavailable

## Run Locally

```bash
cd web_control
python -m pip install -r requirements.txt
python app.py
```

Open:

```text
http://127.0.0.1:5000
```

## Run On Jetson

```bash
cd ~/robotArm/web_control
python3 -m pip install -r requirements.txt
python3 app.py --host 0.0.0.0 --port 5000
```

Open from a computer on the same network:

```text
http://<Jetson-IP>:5000
```

## Optional Mediapipe

Gesture recognition requires Mediapipe. If it is not installed, the page still runs and reports gesture recognition as unavailable.

```bash
python3 -m pip install mediapipe
```

## Safety

- The app uses a single `ActionManager` lock so chat, buttons, gesture recognition, and face tracking do not command the arm at the same time.
- When `Arm_Lib` is missing, the app automatically runs in simulation mode.
- Do not run this together with other programs that control the arm or occupy `/dev/video0`.

