#!/usr/bin/env python3
"""DOFBOT six-servo visual web controller.

Runs on the Jetson Nano. Uses only Python stdlib plus Yahboom Arm_Lib.
"""
import json
import os
import socketserver
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, "/home/jetson/Dofbot/0.py_install/Arm_Lib")
from Arm_Lib import Arm_Device

try:
    from track2_game_ai import (
        EMPTY as TRACK2_EMPTY,
        BLUE as TRACK2_BLUE,
        YELLOW as TRACK2_YELLOW,
        Track2AI,
        apply_move as track2_apply_move,
        board_to_text as track2_board_to_text,
        detect_tamper as track2_detect_tamper,
        other as track2_other,
        winner as track2_winner,
    )
    TRACK2_IMPORT_ERROR = None
except Exception as exc:
    TRACK2_EMPTY = 0
    TRACK2_BLUE = 1
    TRACK2_YELLOW = 2
    Track2AI = None
    track2_apply_move = None
    track2_board_to_text = None
    track2_detect_tamper = None
    track2_other = None
    track2_winner = None
    TRACK2_IMPORT_ERROR = exc

HOST = "0.0.0.0"
PORT = 8088
SERVO_MIN = [0, 0, 0, 0, 0, 0]
SERVO_MAX = [180, 180, 180, 180, 270, 180]
HOME = [90, 90, 90, 90, 270, 90]
MOVE_MS_DEFAULT = 500
CAMERA_ID = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 20
JPEG_QUALITY = 80
CALIBRATION_PATH = "/home/jetson/dofbot_board_view.json"
BOARD_CELL_POSE_PATH = "/home/jetson/dofbot_cell_poses.json"
BOARD_SCAN_MOVE_MS = 550
BOARD_SCAN_SETTLE_SEC = 0.28
BOARD_SCORE_OK = 820
ACTIVE_SCAN_MOVE_MS = 560
ACTIVE_SCAN_SETTLE_SEC = 0.55
ACTIVE_SCAN_AFTER_SAMPLE_SEC = 0.10
ACTIVE_SCAN_SAMPLE_FRAMES = 5
ACTIVE_SCAN_SAMPLE_INTERVAL_SEC = 0.08
ACTIVE_SCAN_SAMPLE_PAD = 0.18
FIXED_SCAN_MIN_CONFIDENCE = 0.12
FIXED_BOARD_CENTER_POSE = [87, 114, 0, 2, 86, 89]
FIXED_BOARD_SCAN_TARGETS = [
    {"row": 1, "col": 1, "label": "左上", "pose": [101, 68, 28, 29, 86, 89]},
    {"row": 1, "col": 2, "label": "中上", "pose": [88, 68, 28, 27, 86, 89]},
    {"row": 1, "col": 3, "label": "右上", "pose": [74, 68, 28, 28, 86, 89]},
    {"row": 2, "col": 1, "label": "左中", "pose": [106, 102, 0, 15, 86, 89]},
    {"row": 2, "col": 2, "label": "中间", "pose": [87, 114, 0, 2, 86, 89]},
    {"row": 2, "col": 3, "label": "右中", "pose": [72, 102, 0, 15, 86, 89]},
    {"row": 3, "col": 1, "label": "左下", "pose": [110, 99, 0, 0, 86, 89]},
    {"row": 3, "col": 2, "label": "中下", "pose": [88, 94, 0, 0, 86, 89]},
    {"row": 3, "col": 3, "label": "右下", "pose": [62, 94, 0, 4, 86, 89]},
]

PICK_SAFE_POSE_5 = [90, 80, 50, 50, 265]
PICK_HOME_POSE_5 = [90, 100, 0, 20, 265]
PICK_CENTER_POSE_5 = [90, 48, 35, 30, 270]
PICK_START_POSE_5 = [180, 61, 19, 21, 265]
GRIPPER_OPEN = 60
GRIPPER_RELEASE = 30
GRIPPER_CLOSE = 135
PICK_MOVE_MS = 1000
PICK_SETTLE_SEC = 0.12
PICK_VISION_ANGLE_ENABLED = False
PICK_VISION_CROP_PAD = 0.26
PICK_VISION_MIN_AREA_RATIO = 0.003
PICK_VISION_MAX_AREA_RATIO = 0.70
PICK_VISION_CENTER_TOLERANCE = 0.75
PICK_VISION_WRIST_LIMIT = 45
PICK_VISION_WRIST_SIGN = -1
PICK_VISION_CENTER_OK_PIXELS = 35.0
PICK_VISION_SEARCH_ENABLED = False
PICK_VISION_SEARCH_MOVE_MS = 320
PICK_VISION_SEARCH_SETTLE_SEC = 0.10
PICK_VISION_SEARCH_OFFSETS = [
    (0, 0),
    (-4, 0), (4, 0),
    (-8, 0), (8, 0),
    (-4, -3), (-4, 3),
    (4, -3), (4, 3),
]
PICK_BOARD_ALIGN_ENABLED = False
PICK_BOARD_ALIGN_REFERENCE_BASE = 90
PICK_BOARD_ALIGN_REFERENCE_WRIST = 265
PICK_BOARD_ALIGN_WRIST_MIN = 235
PICK_BOARD_ALIGN_WRIST_MAX = 270
BOARD_PLACE_POSE_OFFSETS = {}
PICK_LOCATIONS = {
    "start_pose": {"label": "起点", "pose": PICK_START_POSE_5[:]},
    "layer_4": {"label": "Block layer 4", "pose": [90, 70, 57, 12, 270]},
    "layer_3": {"label": "Block layer 3", "pose": [90, 65, 50, 17, 270]},
    "layer_2": {"label": "Block layer 2", "pose": [90, 70, 27, 30, 270]},
    "layer_1": {"label": "Block layer 1", "pose": [90, 60, 27, 36, 270]},
    "yellow": {"label": "Yellow area", "pose": [64, 28, 66, 45, 270]},
    "red": {"label": "Red area", "pose": [117, 28, 66, 45, 270]},
    "green": {"label": "Green area", "pose": [136, 66, 25, 20, 270]},
    "blue": {"label": "Blue area", "pose": [44, 66, 25, 20, 270]},
    "blue_2": {"label": "Blue area 2", "pose": [44, 85, 15, 20, 270]},
}
BOARD_CELL_LABELS = {
    "cell_11": "左上",
    "cell_12": "上中",
    "cell_13": "右上",
    "cell_21": "左中",
    "cell_22": "中间",
    "cell_23": "右中",
    "cell_31": "左下",
    "cell_32": "下中",
    "cell_33": "右下",
}
for _target in FIXED_BOARD_SCAN_TARGETS:
    _key = "cell_%d%d" % (_target["row"], _target["col"])
    PICK_LOCATIONS[_key] = {
        "label": BOARD_CELL_LABELS.get(_key, "Board %d,%d" % (_target["row"], _target["col"])),
        "pose": _target["pose"][:5],
    }
PICK_LOCATIONS["cell_22"]["pose"] = PICK_CENTER_POSE_5[:]
PICK_LOCATIONS["cell_21"]["pose"] = [106, 48, 35, 30, 270]
PICK_LOCATIONS["cell_23"]["pose"] = [72, 48, 35, 30, 270]
BOARD_CELL_KEYS = [
    "cell_11", "cell_12", "cell_13",
    "cell_21", "cell_22", "cell_23",
    "cell_31", "cell_32", "cell_33",
]
FIXED_BOARD_CELL_POSES = {
    "cell_11": {"pick": [103, 16, 82, 33, 270], "place": [103, 16, 82, 33, 270]},
    "cell_12": {"pick": [90, 23, 71, 33, 270], "place": [90, 23, 71, 33, 270]},
    "cell_13": {"pick": [76, 16, 82, 33, 270], "place": [76, 16, 82, 33, 270]},
    "cell_21": {"pick": [106, 53, 26, 36, 254], "place": [106, 56, 26, 36, 254]},
    "cell_22": {"pick": [90, 53, 26, 36, 270], "place": [90, 56, 26, 35, 270]},
    "cell_23": {"pick": [72, 53, 26, 36, 270], "place": [72, 56, 26, 36, 270]},
    "cell_31": {"pick": [114, 69, 9, 27, 248], "place": [114, 69, 9, 27, 248]},
    "cell_32": {"pick": [90, 77, 0, 27, 270], "place": [90, 77, 0, 27, 270]},
    "cell_33": {"pick": [68, 69, 9, 27, 270], "place": [68, 69, 9, 27, 270]},
}

def apply_fixed_board_cell_poses():
    for key in BOARD_CELL_KEYS:
        item = PICK_LOCATIONS.get(key)
        fixed = FIXED_BOARD_CELL_POSES.get(key)
        if not item or not fixed:
            continue
        item["pose"] = fixed["pick"][:]
        item["fixed_pick_pose"] = fixed["pick"][:]
        item["fixed_place_pose"] = fixed["place"][:]

apply_fixed_board_cell_poses()

Arm = Arm_Device()
time.sleep(0.2)
current_angles = HOME[:]
arm_lock = threading.Lock()
track2_lock = threading.Lock()
TRACK2_STATE = {
    "configured": False,
    "started": False,
    "order": "first",
    "ai_color": TRACK2_BLUE,
    "first_player": TRACK2_BLUE,
    "last_board": None,
    "move_index": 0,
    "last_move": None,
}

camera_lock = threading.Lock()
camera_stop_event = threading.Event()
camera_thread = None
latest_frame = None
latest_frame_time = 0.0
latest_frame_error = None
cv2_module = None

DANCE_POSES = [
    ([60, 115, 75, 60, 70, 65], (255, 0, 0)),
    ([120, 65, 115, 120, 250, 125], (0, 255, 0)),
    ([45, 110, 70, 105, 230, 70], (0, 0, 255)),
    ([135, 70, 120, 65, 90, 120], (255, 180, 0)),
    ([70, 120, 85, 125, 245, 60], (180, 0, 255)),
    ([110, 60, 125, 55, 80, 130], (0, 180, 255)),
    ([150, 95, 65, 115, 210, 80], (255, 80, 80)),
    ([30, 85, 130, 70, 120, 125], (80, 255, 160)),
]

HTML = r'''<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>DOFBOT 六电机角度控制</title>
  <style>
    :root {
      color-scheme: light;
      --bg: #eef2f7;
      --panel: #ffffff;
      --ink: #172033;
      --muted: #657084;
      --line: #d9e0ea;
      --accent: #2563eb;
      --ok: #0f9f6e;
      --warn: #d97706;
      --danger: #dc2626;
      --shadow: 0 18px 45px rgba(28, 39, 64, 0.12);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: var(--ink);
      background: var(--bg);
    }
    main { width: min(1180px, calc(100vw - 32px)); margin: 0 auto; padding: 28px 0 36px; }
    header { display: flex; align-items: flex-end; justify-content: space-between; gap: 20px; margin-bottom: 22px; }
    h1 { margin: 0 0 6px; font-size: 30px; line-height: 1.1; letter-spacing: 0; }
    .subtitle { margin: 0; color: var(--muted); font-size: 14px; }
    .connection {
      display: flex; align-items: center; gap: 10px; padding: 10px 12px;
      background: var(--panel); border: 1px solid var(--line); border-radius: 8px;
      box-shadow: var(--shadow); white-space: nowrap;
    }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--danger); box-shadow: 0 0 0 4px rgba(220, 38, 38, 0.12); }
    .dot.connected { background: var(--ok); box-shadow: 0 0 0 4px rgba(15, 159, 110, 0.14); }
    .grid { display: grid; grid-template-columns: minmax(0, 1fr) 390px; gap: 18px; align-items: start; }
    .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; box-shadow: var(--shadow); }
    .toolbar { display: grid; grid-template-columns: auto auto auto auto auto 1fr; gap: 10px; padding: 14px; border-bottom: 1px solid var(--line); align-items: center; }
    button {
      height: 38px; border: 1px solid transparent; border-radius: 6px; padding: 0 13px;
      font: inherit; font-weight: 700; color: #fff; background: var(--accent); cursor: pointer;
    }
    button.secondary { color: var(--ink); background: #f8fafc; border-color: var(--line); }
    button.warning { background: var(--warn); }
    button:disabled { opacity: 0.48; cursor: not-allowed; }
    .hint { justify-self: end; color: var(--muted); font-size: 13px; }
    .servo-list { display: grid; gap: 12px; padding: 14px; }
    .servo {
      display: grid; grid-template-columns: 94px minmax(180px, 1fr) 78px 66px;
      gap: 12px; align-items: center; padding: 12px; border: 1px solid var(--line);
      border-radius: 8px; background: #fbfcfe;
    }
    .servo-name { font-weight: 800; line-height: 1.15; }
    .servo-name span { display: block; margin-top: 3px; color: var(--muted); font-size: 12px; font-weight: 600; }
    input[type="range"] { width: 100%; accent-color: var(--accent); }
    input[type="number"] {
      width: 100%; height: 38px; border: 1px solid var(--line); border-radius: 6px;
      padding: 0 10px; font: inherit; text-align: right; color: var(--ink); background: #fff;
    }
    .manual-pose { border-top: 1px solid var(--line); padding: 14px; display: grid; gap: 12px; }
    .manual-head { display: flex; align-items: center; justify-content: space-between; gap: 10px; }
    .manual-head strong { font-size: 14px; }
    .manual-state { color: var(--muted); font-size: 12px; white-space: nowrap; }
    .manual-grid { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 10px; }
    .manual-field { display: grid; gap: 5px; }
    .manual-field label { color: var(--muted); font-size: 12px; font-weight: 800; }
    .manual-actions { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
    .manual-status { color: var(--muted); font-size: 12px; line-height: 1.5; min-height: 18px; }
    .param-pose-block { display: grid; gap: 8px; }
    .param-pose-title { color: var(--muted); font-size: 12px; font-weight: 800; }
    .param-cell-presets { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
    .param-cell-btn {
      height: auto; min-height: 50px; padding: 7px 8px; display: grid; gap: 3px; align-content: center;
      color: var(--ink); background: #fff; border-color: var(--line); text-align: center; touch-action: manipulation;
    }
    .param-cell-btn strong { font-size: 13px; line-height: 1.1; }
    .param-cell-btn span {
      min-width: 0; overflow: hidden; text-overflow: ellipsis; white-space: nowrap;
      color: var(--muted); font: 10px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }
    .param-cell-btn.active { border-color: var(--accent); background: #eff6ff; box-shadow: 0 0 0 2px rgba(37, 99, 235, 0.14); }
    .param-io { display: grid; grid-template-columns: minmax(0, 1fr) auto auto; gap: 8px; align-items: center; }
    .param-io input {
      width: 100%; height: 38px; border: 1px solid var(--line); border-radius: 6px;
      padding: 0 10px; font: 13px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      text-align: left; color: var(--ink); background: #fff;
    }
    .param-io button { padding: 0 10px; }
    .dial { width: 58px; aspect-ratio: 1; border: 1px solid var(--line); border-radius: 50%; position: relative; background: radial-gradient(circle at center, #fff 0 46%, #e9eef7 47% 100%); }
    .needle { position: absolute; left: 50%; top: 50%; width: 3px; height: 23px; background: var(--accent); border-radius: 999px; transform-origin: 50% 100%; transform: translate(-50%, -100%) rotate(0deg); }
    .side { display: grid; gap: 14px; padding: 16px; }
    .camera-panel, .arm-preview { border: 1px solid var(--line); border-radius: 8px; background: #f8fafc; overflow: hidden; }
    .camera-head { display: flex; align-items: center; justify-content: space-between; gap: 10px; padding: 10px 12px; border-bottom: 1px solid var(--line); }
    .camera-head strong { font-size: 14px; }
    .camera-state { color: var(--muted); font-size: 12px; white-space: nowrap; }
    .camera-frame { aspect-ratio: 4 / 3; background: #111827; display: grid; place-items: center; }
    .camera-frame img { width: 100%; height: 100%; object-fit: contain; display: block; }
    .board-scan { border: 1px solid var(--line); border-radius: 8px; background: #f8fafc; overflow: hidden; }
    .board-head { display: flex; align-items: center; justify-content: space-between; gap: 10px; padding: 10px 12px; border-bottom: 1px solid var(--line); }
    .board-head strong { font-size: 14px; }
    .board-head button { height: 32px; padding: 0 10px; font-size: 13px; }
    .board-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; padding: 12px; }
    .board-cell {
      aspect-ratio: 1; border: 2px solid #111827; border-radius: 6px; display: grid; place-items: center;
      color: #111827; font-weight: 900; font-size: 18px; background: #ffffff;
    }
    .board-cell.yellow { background: #facc15; }
    .board-cell.blue { background: #3b82f6; color: #ffffff; }
    .board-cell.black { background: #111827; color: #ffffff; }
    .board-cell.red { background: #ef4444; color: #ffffff; }
    .board-cell.green { background: #22c55e; color: #ffffff; }
    .board-cell.empty { background: #ffffff; color: #64748b; }
    .board-summary { padding: 0 12px 12px; color: var(--muted); font-size: 12px; line-height: 1.5; }
    .pick-place, .track2-panel { border: 1px solid var(--line); border-radius: 8px; background: #f8fafc; overflow: hidden; }
    .pick-head { padding: 10px 12px; border-bottom: 1px solid var(--line); }
    .pick-head strong { font-size: 14px; }
    .pick-body { display: grid; gap: 10px; padding: 12px; }
    .pick-row { display: grid; grid-template-columns: 58px minmax(0, 1fr); gap: 8px; align-items: center; }
    .pick-row label { color: var(--muted); font-size: 12px; font-weight: 800; }
    .pick-row select {
      width: 100%; height: 38px; border: 1px solid var(--line); border-radius: 6px;
      padding: 0 10px; font: inherit; color: var(--ink); background: #fff;
    }
    .pick-actions { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .track2-actions { display: grid; grid-template-columns: repeat(2, 1fr); gap: 8px; }
    .track2-actions button { min-height: 38px; }
    .track2-monitor { display: grid; gap: 8px; }
    .track2-monitor-grid { padding: 0; }
    .track2-monitor-grid .board-cell { font-size: 13px; line-height: 1.15; padding: 4px; text-align: center; }
    .pick-status { color: var(--muted); font-size: 12px; line-height: 1.5; min-height: 18px; }
    .pick-mode { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    .pick-mode button { height: 34px; font-size: 13px; }
    .pick-mode button.active { background: var(--ok); color: #fff; }
    .pick-board { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; }
    .pick-cell {
      min-height: 46px; height: auto; padding: 6px; border: 2px solid var(--line);
      background: #fff; color: var(--ink); font-size: 13px; line-height: 1.15;
    }
    .pick-cell.from { border-color: var(--ok); background: #dcfce7; }
    .pick-cell.to { border-color: var(--accent); background: #dbeafe; }
    .arm-preview { height: 260px; display: grid; place-items: center; }
    svg { width: 100%; height: 100%; }
    .readout { margin-top: 14px; display: grid; gap: 8px; }
    .kv { display: flex; justify-content: space-between; gap: 10px; padding: 9px 0; border-bottom: 1px solid var(--line); color: var(--muted); font-size: 13px; }
    .kv strong { color: var(--ink); }
    .log { height: 150px; margin-top: 14px; padding: 10px; overflow: auto; border: 1px solid var(--line); border-radius: 8px; background: #0f172a; color: #d7e3f7; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size: 12px; line-height: 1.45; }
    @media (max-width: 920px) { header, .grid { display: block; } .connection { margin-top: 14px; width: fit-content; } .toolbar { grid-template-columns: 1fr 1fr; } .hint { justify-self: start; grid-column: 1 / -1; } .side { margin-top: 18px; } }
    @media (max-width: 620px) { main { width: min(100vw - 20px, 1180px); padding-top: 18px; } h1 { font-size: 24px; } .toolbar { grid-template-columns: 1fr; } .servo { grid-template-columns: 1fr 66px; } .servo input[type="range"] { grid-column: 1 / -1; } .servo .dial { justify-self: end; } .manual-grid, .manual-actions, .param-io { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <main>
    <header>
      <div>
        <h1>DOFBOT 六电机角度控制</h1>
        <p class="subtitle">滑杆和数值框同步调整角度，发送到 Jetson 上的控制服务。</p>
      </div>
      <div class="connection" aria-live="polite"><span id="statusDot" class="dot"></span><strong id="statusText">检测中</strong></div>
    </header>
    <section class="grid">
      <div class="panel">
        <div class="toolbar">
          <button id="readBtn">读取角度</button>
          <button id="homeBtn" class="warning">归中</button>
          <button id="danceBtn">一键跳舞</button>
          <button id="boardViewBtn">获取棋盘视角</button>
          <button id="stopBtn" class="secondary">停止发送</button>
          <span class="hint">接口：<code>/api/angles</code></span>
        </div>
        <div id="servoList" class="servo-list"></div>
        <div class="manual-pose">
          <div class="manual-head">
            <strong>参数执行</strong>
            <span id="manualPoseState" class="manual-state">未确认</span>
          </div>
          <div id="manualPoseInputs" class="manual-grid"></div>
          <div class="param-io">
            <input id="manualPoseText" type="text" value="[90, 90, 90, 90, 270, 90]" />
            <button id="manualImportBtn" class="secondary">导入</button>
            <button id="manualExportBtn" class="secondary">导出</button>
          </div>
          <div class="manual-actions">
            <button id="manualFillBtn" class="secondary">填入当前</button>
            <button id="manualConfirmBtn" class="secondary">确认参数</button>
            <button id="manualRunBtn" disabled>执行到参数</button>
          </div>
          <div id="manualPoseStatus" class="manual-status">填入 6 个电机角度后先确认，再执行。</div>
        </div>
        <div class="manual-pose">
          <div class="manual-head">
            <strong>参数抓放</strong>
            <span id="paramPickPlaceState" class="manual-state">待执行</span>
          </div>
          <div class="param-pose-block">
            <div class="param-pose-title">抓取按钮</div>
            <div id="paramPickPresets" class="param-cell-presets" aria-label="pick parameter presets"></div>
          </div>
          <div class="param-pose-block">
            <div class="param-pose-title">抓取参数 1-5 轴</div>
            <div id="paramPickInputs" class="manual-grid"></div>
            <div class="param-io">
              <input id="paramPickText" type="text" value="[90, 53, 26, 36, 270]" />
              <button id="paramPickImportBtn" class="secondary">导入</button>
              <button id="paramPickExportBtn" class="secondary">导出</button>
            </div>
          </div>
          <div class="param-pose-block">
            <div class="param-pose-title">放置按钮</div>
            <div id="paramPlacePresets" class="param-cell-presets" aria-label="place parameter presets"></div>
          </div>
          <div class="param-pose-block">
            <div class="param-pose-title">放置参数 1-5 轴</div>
            <div id="paramPlaceInputs" class="manual-grid"></div>
            <div class="param-io">
              <input id="paramPlaceText" type="text" value="[90, 56, 26, 35, 270]" />
              <button id="paramPlaceImportBtn" class="secondary">导入</button>
              <button id="paramPlaceExportBtn" class="secondary">导出</button>
            </div>
          </div>
          <div class="manual-actions">
            <button id="paramFillDefaultBtn" class="secondary">填入当前方案</button>
            <button id="paramRunBtn">抓取并放置</button>
          </div>
          <div id="paramPickPlaceStatus" class="manual-status">输入抓取和放置 1-5 轴参数，第 6 轴夹爪自动控制。</div>
        </div>
      </div>
      <aside class="panel side">
        <div class="camera-panel">
          <div class="camera-head">
            <strong>摄像头实时画面</strong>
            <span id="cameraState" class="camera-state">加载中</span>
          </div>
          <div class="camera-frame">
            <img id="cameraFeed" src="/video_feed" alt="Jetson 摄像头实时画面" />
          </div>
        </div>
        <div class="board-scan">
          <div class="board-head">
            <strong>棋盘扫描</strong>
            <button id="scanBoardBtn">扫描棋盘</button>
          </div>
          <div id="boardGrid" class="board-grid"></div>
          <div id="boardSummary" class="board-summary">尚未扫描</div>
        </div>
        <div class="track2-panel">
          <div class="pick-head"><strong>赛道2下棋</strong></div>
          <div class="pick-body">
            <div class="pick-row">
              <label for="track2Order">顺序</label>
              <select id="track2Order">
                <option value="first">先手</option>
                <option value="second">后手</option>
              </select>
            </div>
            <div class="pick-row">
              <label for="track2Color">颜色</label>
              <select id="track2Color">
                <option value="blue">蓝色</option>
                <option value="yellow">黄色</option>
              </select>
            </div>
            <div class="track2-actions">
              <button id="track2PrepareStartBtn" class="secondary">到起点</button>
              <button id="track2StartBtn">开始下棋</button>
              <button id="track2NextBtn" class="secondary">下一步</button>
              <button id="track2ResetBtn" class="secondary">重置棋局</button>
            </div>
            <div class="track2-monitor">
              <div class="param-pose-title">棋盘实时监控</div>
              <div id="track2MonitorGrid" class="board-grid track2-monitor-grid"></div>
              <div id="track2MonitorSummary" class="pick-status">我方：无；对方：无。</div>
            </div>
            <div id="track2Status" class="pick-status">选择先后手和颜色；先手可先到起点，放好棋子后开始。</div>
          </div>
        </div>
        <div class="pick-place">
          <div class="pick-head"><strong>Pick and place</strong></div>
          <div class="pick-body">
            <div class="pick-mode">
              <button id="pickModeFrom" class="secondary active">Set From</button>
              <button id="pickModeTo" class="secondary">Set To</button>
            </div>
            <div id="pickBoard" class="pick-board"></div>
            <div class="pick-row">
              <label for="pickSource">From</label>
              <select id="pickSource"></select>
            </div>
            <div class="pick-row">
              <label for="pickTarget">To</label>
              <select id="pickTarget"></select>
            </div>
            <div class="pick-actions">
              <button id="pickPlaceBtn">Run move</button>
              <button id="pickCenterBtn" class="secondary">Center grab test</button>
              <button id="placeCenterBtn" class="secondary">Center place test</button>
              <button id="cycleCenterBtn" class="secondary">Center pick+place</button>
              <button id="midLeftToRightBtn" class="secondary">中左 -> 中右</button>
              <button id="midRightToLeftBtn" class="secondary">中右 -> 中左</button>
              <button id="pickRefreshBtn" class="secondary">Reload points</button>
            </div>
            <div id="pickStatus" class="pick-status">Select a source block and destination.</div>
          </div>
        </div>
        <div class="arm-preview">
          <svg viewBox="0 0 320 320" role="img" aria-label="机械臂角度示意图">
            <line x1="160" y1="278" x2="160" y2="238" stroke="#64748b" stroke-width="16" stroke-linecap="round" />
            <g id="armBase" transform="translate(160 238)">
              <line x1="0" y1="0" x2="0" y2="-70" stroke="#2563eb" stroke-width="14" stroke-linecap="round" />
              <g id="joint2" transform="translate(0 -70)">
                <line x1="0" y1="0" x2="0" y2="-58" stroke="#0f9f6e" stroke-width="12" stroke-linecap="round" />
                <g id="joint3" transform="translate(0 -58)">
                  <line x1="0" y1="0" x2="0" y2="-44" stroke="#d97706" stroke-width="10" stroke-linecap="round" />
                  <circle cx="0" cy="-52" r="9" fill="#dc2626" />
                </g>
              </g>
              <circle cx="0" cy="0" r="15" fill="#172033" />
              <circle cx="0" cy="-70" r="12" fill="#172033" />
              <circle cx="0" cy="-128" r="10" fill="#172033" />
            </g>
            <text x="160" y="304" text-anchor="middle" fill="#64748b" font-size="12">简化姿态预览</text>
          </svg>
        </div>
        <div class="readout" id="readout"></div>
        <div class="log" id="log"></div>
      </aside>
    </section>
  </main>
<script>
const servoNames = ['1 底座', '2 肩部', '3 肘部', '4 腕部', '5 旋腕', '6 夹爪'];
const minAngles = [0, 0, 0, 0, 0, 0];
const maxAngles = [180, 180, 180, 180, 270, 180];
const homeAngles = [90, 90, 90, 90, 270, 90];
const state = { angles: homeAngles.slice(), connected: false, sendTimer: null, sending: true, pickMode: 'from', manualConfirmed: null, paramPickPresetKey: 'cell_22', paramPlacePresetKey: 'cell_22' };
const servoList = document.getElementById('servoList');
const manualPoseInputs = document.getElementById('manualPoseInputs');
const manualPoseState = document.getElementById('manualPoseState');
const manualPoseStatus = document.getElementById('manualPoseStatus');
const manualRunBtn = document.getElementById('manualRunBtn');
const manualPoseText = document.getElementById('manualPoseText');
const paramPickInputs = document.getElementById('paramPickInputs');
const paramPlaceInputs = document.getElementById('paramPlaceInputs');
const paramPickPlaceState = document.getElementById('paramPickPlaceState');
const paramPickPlaceStatus = document.getElementById('paramPickPlaceStatus');
const paramRunBtn = document.getElementById('paramRunBtn');
const paramPickText = document.getElementById('paramPickText');
const paramPlaceText = document.getElementById('paramPlaceText');
const paramPickPresets = document.getElementById('paramPickPresets');
const paramPlacePresets = document.getElementById('paramPlacePresets');
const readout = document.getElementById('readout');
const logEl = document.getElementById('log');
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const cameraFeed = document.getElementById('cameraFeed');
const cameraState = document.getElementById('cameraState');
const boardGrid = document.getElementById('boardGrid');
const boardSummary = document.getElementById('boardSummary');
const pickSource = document.getElementById('pickSource');
const pickTarget = document.getElementById('pickTarget');
const pickStatus = document.getElementById('pickStatus');
const pickBoard = document.getElementById('pickBoard');
const pickModeFrom = document.getElementById('pickModeFrom');
const pickModeTo = document.getElementById('pickModeTo');
const track2Order = document.getElementById('track2Order');
const track2Color = document.getElementById('track2Color');
const track2PrepareStartBtn = document.getElementById('track2PrepareStartBtn');
const track2StartBtn = document.getElementById('track2StartBtn');
const track2NextBtn = document.getElementById('track2NextBtn');
const track2ResetBtn = document.getElementById('track2ResetBtn');
const track2MonitorGrid = document.getElementById('track2MonitorGrid');
const track2MonitorSummary = document.getElementById('track2MonitorSummary');
const track2Status = document.getElementById('track2Status');
const boardPickCells = [
  { key: 'cell_11', label: '左上' },
  { key: 'cell_12', label: '上中' },
  { key: 'cell_13', label: '右上' },
  { key: 'cell_21', label: '左中' },
  { key: 'cell_22', label: '中间' },
  { key: 'cell_23', label: '右中' },
  { key: 'cell_31', label: '左下' },
  { key: 'cell_32', label: '下中' },
  { key: 'cell_33', label: '右下' },
];
const paramBoardPosePresets = [
  { key: 'start_pose', label: '\u8d77\u70b9', pick: [180, 61, 19, 21, 265], place: [180, 61, 19, 21, 265] },
  { key: 'cell_11', label: '\u5de6\u4e0a', pick: [103, 16, 82, 33, 270], place: [103, 16, 82, 33, 270] },
  { key: 'cell_12', label: '\u4e2d\u4e0a', pick: [90, 23, 71, 33, 270], place: [90, 23, 71, 33, 270] },
  { key: 'cell_13', label: '\u53f3\u4e0a', pick: [76, 16, 82, 33, 270], place: [76, 16, 82, 33, 270] },
  { key: 'cell_21', label: '\u4e2d\u5de6', pick: [106, 53, 26, 36, 254], place: [106, 56, 26, 36, 254] },
  { key: 'cell_22', label: '\u4e2d\u95f4', pick: [90, 53, 26, 36, 270], place: [90, 56, 26, 35, 270] },
  { key: 'cell_23', label: '\u4e2d\u53f3', pick: [72, 53, 26, 36, 270], place: [72, 56, 26, 36, 270] },
  { key: 'cell_31', label: '\u5de6\u4e0b', pick: [114, 69, 9, 27, 248], place: [114, 69, 9, 27, 248] },
  { key: 'cell_32', label: '\u4e2d\u4e0b', pick: [90, 77, 0, 27, 270], place: [90, 77, 0, 27, 270] },
  { key: 'cell_33', label: '\u53f3\u4e0b', pick: [68, 69, 9, 27, 270], place: [68, 69, 9, 27, 270] },
];
const currentPickPreset = { key: 'current_pose', label: '\u5f53\u524d\u4f4d\u7f6e', pick: null, place: null };
function log(message) { const time = new Date().toLocaleTimeString(); logEl.textContent += `[${time}] ${message}\n`; logEl.scrollTop = logEl.scrollHeight; }
function clampAngle(value, i) { const n = Number(value); if (!Number.isFinite(n)) return homeAngles[i]; return Math.max(minAngles[i], Math.min(maxAngles[i], Math.round(n))); }
function api(path, options) { return fetch(path, Object.assign({ headers: { 'Content-Type': 'application/json' } }, options || {})).then(async res => { const data = await res.json(); if (!res.ok || data.status === 'error') throw new Error(data.message || res.statusText); return data; }); }
function formatParamArray(values) { return `[${values.map(value => Math.round(Number(value))).join(', ')}]`; }
function parseParamArray(text, expectedLength, label) {
  let values;
  try {
    values = JSON.parse(String(text).trim());
  } catch (err) {
    values = String(text).match(/-?\d+(?:\.\d+)?/g);
  }
  if (!Array.isArray(values) || values.length !== expectedLength) {
    throw new Error(`${label}需要 ${expectedLength} 个参数，格式如 [72, 53, 26, 36, 270]`);
  }
  return values.map((value, index) => {
    const number = Number(value);
    if (!Number.isFinite(number)) throw new Error(`${label}第 ${index + 1} 个参数不是有效数字`);
    if (number < minAngles[index] || number > maxAngles[index]) throw new Error(`${label}第 ${index + 1} 个参数超出范围 ${minAngles[index]}-${maxAngles[index]}`);
    return Math.round(number);
  });
}
function setInputValues(container, values) {
  Array.from(container.querySelectorAll('input')).forEach((input, index) => {
    input.value = values[index];
  });
}
function renderServos() {
  servoList.innerHTML = '';
  state.angles.forEach((angle, index) => {
    const row = document.createElement('div');
    row.className = 'servo';
    row.innerHTML = `<div class="servo-name">${servoNames[index]}<span>${minAngles[index]}-${maxAngles[index]} deg</span></div><input type="range" min="${minAngles[index]}" max="${maxAngles[index]}" value="${angle}" data-index="${index}" aria-label="${servoNames[index]}角度" /><input type="number" min="${minAngles[index]}" max="${maxAngles[index]}" value="${angle}" data-index="${index}" aria-label="${servoNames[index]}角度数值" /><div class="dial" aria-hidden="true"><span class="needle"></span></div>`;
    servoList.appendChild(row);
  });
  servoList.querySelectorAll('input').forEach(input => input.addEventListener('input', event => setAngle(Number(event.target.dataset.index), event.target.value, true)));
  updateVisuals();
}
function renderManualPoseInputs() {
  manualPoseInputs.innerHTML = servoNames.map((name, index) => `<div class="manual-field"><label for="manualAngle${index}">${name}</label><input id="manualAngle${index}" type="number" min="${minAngles[index]}" max="${maxAngles[index]}" value="${state.angles[index]}" data-manual-index="${index}" /></div>`).join('');
  manualPoseInputs.querySelectorAll('input').forEach(input => {
    input.addEventListener('input', () => {
      state.manualConfirmed = null;
      manualRunBtn.disabled = true;
      manualPoseState.textContent = '未确认';
      manualPoseStatus.textContent = '参数已修改，请重新确认。';
    });
  });
}
function renderPose5Inputs(container, prefix, values, presetType) {
  container.innerHTML = servoNames.slice(0, 5).map((name, index) => `<div class="manual-field"><label for="${prefix}${index}">${name}</label><input id="${prefix}${index}" type="number" min="${minAngles[index]}" max="${maxAngles[index]}" value="${values[index]}" data-pose-index="${index}" /></div>`).join('');
  container.querySelectorAll('input').forEach(input => {
    input.addEventListener('input', () => {
      if (presetType === 'pick') state.paramPickPresetKey = null;
      if (presetType === 'place') state.paramPlacePresetKey = null;
      updateParamCellPresetSelection();
    });
  });
}
function renderParamPickPlaceInputs() {
  const pickDefault = [90, 53, 26, 36, 270];
  const placeDefault = [90, 56, 26, 35, 270];
  renderPose5Inputs(paramPickInputs, 'paramPickAngle', pickDefault, 'pick');
  renderPose5Inputs(paramPlaceInputs, 'paramPlaceAngle', placeDefault, 'place');
  paramPickText.value = formatParamArray(pickDefault);
  paramPlaceText.value = formatParamArray(placeDefault);
  state.paramPickPresetKey = 'cell_22';
  state.paramPlacePresetKey = 'cell_22';
  updateParamCellPresetSelection();
}
function setParamPickValues(pickPose) {
  setInputValues(paramPickInputs, pickPose);
  paramPickText.value = formatParamArray(pickPose);
}
function setParamPlaceValues(placePose) {
  setInputValues(paramPlaceInputs, placePose);
  paramPlaceText.value = formatParamArray(placePose);
}
function updateParamCellPresetSelection() {
  [
    { container: paramPickPresets, activeKey: state.paramPickPresetKey },
    { container: paramPlacePresets, activeKey: state.paramPlacePresetKey },
  ].forEach(group => {
    if (!group.container) return;
    group.container.querySelectorAll('[data-param-cell-key]').forEach(button => {
      const active = button.dataset.paramCellKey === group.activeKey;
      button.classList.toggle('active', active);
      button.setAttribute('aria-pressed', active ? 'true' : 'false');
    });
  });
}
function applyParamCellPreset(type, key) {
  if (type === 'pick' && key === currentPickPreset.key) {
    const displayPose = state.angles.slice(0, 5).map((value, index) => clampAngle(value, index));
    setParamPickValues(displayPose);
    state.paramPickPresetKey = currentPickPreset.key;
    updateParamCellPresetSelection();
    paramPickPlaceState.textContent = '\u5f85\u6267\u884c';
    paramPickPlaceStatus.textContent = '\u5df2\u9009\u62e9\u5f53\u524d\u4f4d\u7f6e\u6293\u53d6\uff1a\u6267\u884c\u65f6\u5c06\u76f4\u63a5\u5728\u5f53\u524d\u89d2\u5ea6\u95ed\u5408\u5939\u722a';
    log('\u6293\u53d6\u5bfc\u5165\uff1a\u5f53\u524d\u4f4d\u7f6e\uff0c\u6267\u884c\u65f6\u76f4\u63a5\u95ed\u722a');
    return;
  }
  const preset = paramBoardPosePresets.find(item => item.key === key);
  if (!preset) return;
  if (type === 'pick') {
    const pickPose = preset.pick.slice();
    setParamPickValues(pickPose);
    state.paramPickPresetKey = key;
    paramPickPlaceStatus.textContent = `${preset.label}\u6293\u53d6\u5df2\u5bfc\u5165\uff1a${paramPickText.value}`;
    log(`\u6293\u53d6\u683c\u5b50\u53c2\u6570\u5bfc\u5165\uff1a${preset.label} pick=${paramPickText.value}`);
  } else {
    const placePose = preset.place.slice();
    setParamPlaceValues(placePose);
    state.paramPlacePresetKey = key;
    paramPickPlaceStatus.textContent = `${preset.label}\u653e\u7f6e\u5df2\u5bfc\u5165\uff1a${paramPlaceText.value}`;
    log(`\u653e\u7f6e\u683c\u5b50\u53c2\u6570\u5bfc\u5165\uff1a${preset.label} place=${paramPlaceText.value}`);
  }
  updateParamCellPresetSelection();
  paramPickPlaceState.textContent = '\u5f85\u6267\u884c';
}
function renderParamCellPresets() {
  [
    { container: paramPickPresets, type: 'pick', poseKey: 'pick', presets: [currentPickPreset].concat(paramBoardPosePresets) },
    { container: paramPlacePresets, type: 'place', poseKey: 'place', presets: paramBoardPosePresets },
  ].forEach(group => {
    if (!group.container) return;
    group.container.innerHTML = group.presets.map(preset => {
      const detail = preset.key === currentPickPreset.key ? '\u76f4\u63a5\u95ed\u722a' : formatParamArray(preset[group.poseKey]);
      return `<button type="button" class="param-cell-btn" data-param-cell-key="${preset.key}" aria-pressed="false"><strong>${preset.label}</strong><span>${detail}</span></button>`;
    }).join('');
    group.container.querySelectorAll('[data-param-cell-key]').forEach(button => {
      button.addEventListener('click', () => applyParamCellPreset(group.type, button.dataset.paramCellKey));
    });
  });
  updateParamCellPresetSelection();
}
function readPose5Inputs(container, label) {
  const inputs = Array.from(container.querySelectorAll('input'));
  if (inputs.length !== 5) throw new Error(`${label}需要 5 个参数`);
  return inputs.map((input, index) => {
    const value = Number(input.value);
    if (!Number.isFinite(value)) throw new Error(`${label} ${servoNames[index]} 不是有效数字`);
    if (value < minAngles[index] || value > maxAngles[index]) throw new Error(`${label} ${servoNames[index]} 超出范围 ${minAngles[index]}-${maxAngles[index]}`);
    return Math.round(value);
  });
}
function fillParamPickPlaceDefaults() {
  renderParamPickPlaceInputs();
  paramPickPlaceState.textContent = '待执行';
  paramPickPlaceStatus.textContent = '已填入当前方案：抓取 [90,53,26,36,270]，放置 [90,56,26,35,270]。';
}
function importPose5Text(textInput, container, label, presetType) {
  try {
    const values = parseParamArray(textInput.value, 5, label);
    setInputValues(container, values);
    textInput.value = formatParamArray(values);
    if (presetType === 'pick') state.paramPickPresetKey = null;
    if (presetType === 'place') state.paramPlacePresetKey = null;
    updateParamCellPresetSelection();
    paramPickPlaceState.textContent = '待执行';
    paramPickPlaceStatus.textContent = `${label}已导入：${textInput.value}`;
    log(`${label}参数导入：${textInput.value}`);
  } catch (err) {
    paramPickPlaceState.textContent = '参数错误';
    paramPickPlaceStatus.textContent = err.message;
    log(`${label}参数导入失败：${err.message}`);
  }
}
function exportPose5Text(textInput, container, label) {
  try {
    const values = readPose5Inputs(container, label);
    textInput.value = formatParamArray(values);
    paramPickPlaceStatus.textContent = `${label}已导出：${textInput.value}`;
    log(`${label}参数导出：${textInput.value}`);
  } catch (err) {
    paramPickPlaceStatus.textContent = err.message;
  }
}
function runParamPickPlace() {
  let pickPose;
  let placePose;
  const pickCurrent = state.paramPickPresetKey === currentPickPreset.key;
  try {
    pickPose = pickCurrent ? null : readPose5Inputs(paramPickInputs, '抓取');
    placePose = readPose5Inputs(paramPlaceInputs, '放置');
  } catch (err) {
    paramPickPlaceState.textContent = '参数错误';
    paramPickPlaceStatus.textContent = err.message;
    log(`参数抓放校验失败：${err.message}`);
    return;
  }
  paramRunBtn.disabled = true;
  paramPickPlaceState.textContent = '执行中';
  const pickLabel = pickCurrent ? '\u5f53\u524d\u4f4d\u7f6e\u76f4\u63a5\u95ed\u722a' : pickPose.join(', ');
  paramPickPlaceStatus.textContent = `抓取 ${pickLabel}，放置 ${placePose.join(', ')}`;
  log(`参数抓放开始：pick=${pickLabel} place=${placePose.join(', ')}`);
  api('/api/manual_pick_place', { method: 'POST', body: JSON.stringify({ pick_pose: pickPose, place_pose: placePose, pick_current: pickCurrent }) })
    .then(data => {
      data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      paramPickPlaceState.textContent = '完成';
      paramPickPlaceStatus.textContent = `完成，当前角度：${data.angles.join(', ')}`;
      log('参数抓放完成');
    })
    .catch(err => {
      updateConnection(false);
      paramPickPlaceState.textContent = '失败';
      paramPickPlaceStatus.textContent = `执行失败：${err.message}`;
      log(`参数抓放失败：${err.message}`);
    })
    .finally(() => {
      paramRunBtn.disabled = false;
    });
}
function fillManualFromAngles(angles) {
  manualPoseInputs.querySelectorAll('input').forEach((input, index) => {
    input.value = clampAngle(angles[index], index);
  });
  manualPoseText.value = formatParamArray(Array.from(manualPoseInputs.querySelectorAll('input')).map(input => input.value));
  state.manualConfirmed = null;
  manualRunBtn.disabled = true;
  manualPoseState.textContent = '未确认';
  manualPoseStatus.textContent = '已填入当前显示角度，请确认参数。';
}
function importManualPoseText() {
  try {
    const angles = parseParamArray(manualPoseText.value, 6, '参数执行');
    setInputValues(manualPoseInputs, angles);
    state.manualConfirmed = null;
    manualRunBtn.disabled = true;
    manualPoseState.textContent = '未确认';
    manualPoseStatus.textContent = `已导入：${formatParamArray(angles)}`;
    log(`导入执行参数：${formatParamArray(angles)}`);
  } catch (err) {
    manualPoseStatus.textContent = err.message;
    log(`导入执行参数失败：${err.message}`);
  }
}
function exportManualPoseText() {
  try {
    const angles = readManualAngles();
    manualPoseText.value = formatParamArray(angles);
    manualPoseStatus.textContent = `已导出：${manualPoseText.value}`;
    log(`导出执行参数：${manualPoseText.value}`);
  } catch (err) {
    manualPoseStatus.textContent = err.message;
  }
}
function readManualAngles() {
  const inputs = Array.from(manualPoseInputs.querySelectorAll('input'));
  if (inputs.length !== 6) throw new Error('需要 6 个电机角度');
  return inputs.map((input, index) => {
    const value = Number(input.value);
    if (!Number.isFinite(value)) throw new Error(`${servoNames[index]} 不是有效数字`);
    if (value < minAngles[index] || value > maxAngles[index]) throw new Error(`${servoNames[index]} 超出范围 ${minAngles[index]}-${maxAngles[index]}`);
    return Math.round(value);
  });
}
function confirmManualPose() {
  try {
    const angles = readManualAngles();
    state.manualConfirmed = angles;
    manualRunBtn.disabled = false;
    manualPoseState.textContent = '已确认';
    manualPoseStatus.textContent = `确认参数：${angles.join(', ')}`;
    log(`确认电机参数：${angles.join(', ')}`);
  } catch (err) {
    state.manualConfirmed = null;
    manualRunBtn.disabled = true;
    manualPoseState.textContent = '未确认';
    manualPoseStatus.textContent = err.message;
    log(`参数确认失败：${err.message}`);
  }
}
function runManualPose() {
  const angles = state.manualConfirmed;
  if (!angles) {
    manualPoseStatus.textContent = '请先确认参数。';
    return;
  }
  manualRunBtn.disabled = true;
  manualPoseStatus.textContent = `正在执行：${angles.join(', ')}`;
  log(`执行电机参数：${angles.join(', ')}`);
  api('/api/angles', { method: 'POST', body: JSON.stringify({ angles, time: 800 }) })
    .then(data => {
      data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      manualPoseStatus.textContent = `已执行：${data.angles.join(', ')}`;
      log(`电机参数执行完成：${data.angles.join(', ')}`);
    })
    .catch(err => {
      updateConnection(false);
      manualPoseStatus.textContent = `执行失败：${err.message}`;
      log(`电机参数执行失败：${err.message}`);
    })
    .finally(() => {
      manualRunBtn.disabled = !state.manualConfirmed;
    });
}
function setAngle(index, value, send) {
  state.angles[index] = clampAngle(value, index);
  const range = servoList.querySelector(`input[type="range"][data-index="${index}"]`);
  const number = servoList.querySelector(`input[type="number"][data-index="${index}"]`);
  if (range) range.value = state.angles[index];
  if (number) number.value = state.angles[index];
  updateVisuals();
  if (send && state.sending) scheduleSend();
}
function updateConnection(ok) { state.connected = ok; statusDot.classList.toggle('connected', ok); statusText.textContent = ok ? '已连接' : '未连接'; }
function updateVisuals() {
  servoList.querySelectorAll('.servo').forEach((row, index) => {
    const span = maxAngles[index] - minAngles[index];
    const normalized = ((state.angles[index] - minAngles[index]) / span) * 180 - 90;
    row.querySelector('.needle').style.transform = `translate(-50%, -100%) rotate(${normalized}deg)`;
  });
  readout.innerHTML = state.angles.map((angle, index) => `<div class="kv"><span>${servoNames[index]}</span><strong>${angle} deg</strong></div>`).join('');
  document.getElementById('armBase').setAttribute('transform', `translate(160 238) rotate(${state.angles[0] - 90})`);
  document.getElementById('joint2').setAttribute('transform', `translate(0 -70) rotate(${(state.angles[1] - 90) * 0.8})`);
  document.getElementById('joint3').setAttribute('transform', `translate(0 -58) rotate(${(state.angles[2] - 90) * 0.8})`);
}
function scheduleSend() { window.clearTimeout(state.sendTimer); state.sendTimer = window.setTimeout(sendAngles, 140); }
function sendAngles() {
  api('/api/angles', { method: 'POST', body: JSON.stringify({ angles: state.angles, time: 500 }) })
    .then(data => { updateConnection(true); log(`发送角度：${data.angles.join(', ')}`); })
    .catch(err => { updateConnection(false); log(`发送失败：${err.message}`); });
}
function readAngles() {
  api('/api/angles')
    .then(data => { data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false)); updateConnection(true); log(`读取角度：${state.angles.join(', ')}`); })
    .catch(err => { updateConnection(false); log(`读取失败：${err.message}`); });
}
function home() {
  homeAngles.forEach((angle, i) => setAngle(i, angle, false));
  api('/api/home', { method: 'POST', body: '{}' })
    .then(data => { data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false)); updateConnection(true); log('已发送归中指令'); })
    .catch(err => { updateConnection(false); log(`归中失败：${err.message}`); });
}
function dance() {
  const danceBtn = document.getElementById('danceBtn');
  danceBtn.disabled = true;
  state.sending = false;
  document.getElementById('stopBtn').textContent = '恢复发送';
  log('开始一键跳舞');
  api('/api/dance', { method: 'POST', body: '{}' })
    .then(data => {
      data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      log('跳舞完成，已归中');
    })
    .catch(err => {
      updateConnection(false);
      log(`跳舞失败：${err.message}`);
    })
    .finally(() => {
      danceBtn.disabled = false;
    });
}
function findBoardView() {
  const btn = document.getElementById('boardViewBtn');
  btn.disabled = true;
  state.sending = false;
  document.getElementById('stopBtn').textContent = '恢复发送';
  log('开始自动寻找棋盘视角');
  api('/api/find_board_view', { method: 'POST', body: '{}' })
    .then(data => {
      data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      const okText = data.board.complete ? '完整' : '未完整';
      log(`棋盘视角${okText}：分数 ${Math.round(data.board.score)}，角度 ${data.angles.join(', ')}`);
      log(`已保存棋盘视角到 ${data.saved_to}`);
    })
    .catch(err => {
      updateConnection(false);
      log(`寻找棋盘视角失败：${err.message}`);
    })
    .finally(() => {
      btn.disabled = false;
    });
}
function renderBoardScan(cells) {
  const labels = { '黄色': '黄', '蓝色': '蓝', '黑色': '黑', '红色': '红', '绿色': '绿', '无': '无' };
  const classes = { '黄色': 'yellow', '蓝色': 'blue', '黑色': 'black', '红色': 'red', '绿色': 'green', '无': 'empty' };
  boardGrid.innerHTML = cells.flat().map(cell => `<div class="board-cell ${classes[cell.color] || 'empty'}">${labels[cell.color] || cell.color}</div>`).join('');
}
function scanBoard() {
  const btn = document.getElementById('scanBoardBtn');
  btn.disabled = true;
  log('开始移动扫描整张棋盘');
  api('/api/scan_board')
    .then(data => {
      renderBoardScan(data.cells);
      const rows = data.cells.map((row, i) => `第${i + 1}行：` + row.map(cell => cell.color).join('，')).join('；');
      boardSummary.textContent = `${rows}。扫描 ${data.scan_count || 0} 个视角。`;
      updateConnection(true);
      log(rows);
    })
    .catch(err => {
      updateConnection(false);
      boardSummary.textContent = `扫描失败：${err.message}`;
      log(`扫描棋盘失败：${err.message}`);
    })
    .finally(() => {
      btn.disabled = false;
    });
}
function track2Payload() {
  return { order: track2Order.value, color: track2Color.value };
}
function boardRowsText(cells) {
  return cells.map((row, i) => `第${i + 1}行：` + row.map(cell => cell.color).join('，')).join('；');
}
function setTrack2ButtonsDisabled(disabled) {
  track2PrepareStartBtn.disabled = disabled;
  track2StartBtn.disabled = disabled;
  track2NextBtn.disabled = disabled;
  track2ResetBtn.disabled = disabled;
}
function renderTrack2Monitor(cells, positions, trackState) {
  const labels = { '黄色': '黄', '蓝色': '蓝', '黑色': '黑', '红色': '红', '绿色': '绿', '无': '空' };
  const classes = { '黄色': 'yellow', '蓝色': 'blue', '黑色': 'black', '红色': 'red', '绿色': 'green', '无': 'empty' };
  const flat = cells ? cells.flat() : [];
  track2MonitorGrid.innerHTML = flat.map((cell, index) => {
    const owner = cell.owner || '空';
    const color = labels[cell.color] || cell.color || '空';
    const text = owner === '空' ? `${index + 1}` : `${owner.slice(0, 1)}${color}`;
    const title = `${cell.label || index + 1} ${owner} ${cell.color || '无'}`;
    return `<div class="board-cell ${classes[cell.color] || 'empty'}" title="${title}">${text}</div>`;
  }).join('');
  const own = positions && positions.own && positions.own.length ? positions.own.map(item => `${item.label}(${item.color})`).join('、') : '无';
  const opponent = positions && positions.opponent && positions.opponent.length ? positions.opponent.map(item => `${item.label}(${item.color})`).join('、') : '无';
  const moves = trackState && Number.isFinite(Number(trackState.move_index)) ? `；步数：${trackState.move_index}` : '';
  track2MonitorSummary.textContent = `我方：${own}；对方：${opponent}${moves}`;
}
function loadTrack2State() {
  api('/api/track2_state')
    .then(data => {
      if (data.cells) renderTrack2Monitor(data.cells, data.positions, data.state);
    })
    .catch(() => {});
}
function updateTrack2Result(data, actionLabel) {
  if (data.cells) {
    renderBoardScan(data.cells);
    boardSummary.textContent = `${boardRowsText(data.cells)}。赛道2状态更新。`;
    renderTrack2Monitor(data.cells, data.positions, data.state);
  } else if (data.scan && data.scan.cells) {
    renderBoardScan(data.scan.cells);
    boardSummary.textContent = `${boardRowsText(data.scan.cells)}。扫描 ${data.scan.scan_count || 0} 个视角。`;
  }
  if (data.angles) data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
  updateConnection(true);
  const move = data.move ? `${data.move.kind} ${data.move.src || '起点'} -> ${data.move.dst}` : '无动作';
  const winner = data.winner ? `，胜者：${data.winner}` : '';
  track2Status.textContent = `${actionLabel}完成：${data.message || ''} ${move}${winner}`;
  log(`赛道2${actionLabel}完成：${move}${winner}`);
}
function runTrack2Action(path, actionLabel) {
  setTrack2ButtonsDisabled(true);
  state.sending = false;
  document.getElementById('stopBtn').textContent = '恢复发送';
  track2Status.textContent = `${actionLabel}执行中...`;
  log(`赛道2${actionLabel}开始`);
  api(path, { method: 'POST', body: JSON.stringify(track2Payload()) })
    .then(data => updateTrack2Result(data, actionLabel))
    .catch(err => {
      updateConnection(false);
      track2Status.textContent = `${actionLabel}失败：${err.message}`;
      log(`赛道2${actionLabel}失败：${err.message}`);
    })
    .finally(() => {
      setTrack2ButtonsDisabled(false);
    });
}
function fillPickSelect(select, locations) {
  const current = select.value;
  select.innerHTML = locations.map(item => `<option value="${item.key}">${item.label}</option>`).join('');
  if (locations.some(item => item.key === current)) select.value = current;
}
function setPickMode(mode) {
  state.pickMode = mode === 'to' ? 'to' : 'from';
  pickModeFrom.classList.toggle('active', state.pickMode === 'from');
  pickModeTo.classList.toggle('active', state.pickMode === 'to');
}
function renderPickBoard() {
  pickBoard.innerHTML = boardPickCells.map(cell => {
    const marks = [];
    if (pickSource.value === cell.key) marks.push('from');
    if (pickTarget.value === cell.key) marks.push('to');
    const markText = marks.includes('from') && marks.includes('to') ? 'From/To' : marks.includes('from') ? 'From' : marks.includes('to') ? 'To' : '';
    return `<button type="button" class="pick-cell ${marks.join(' ')}" data-cell="${cell.key}"><strong>${cell.label}</strong>${markText ? `<span>${markText}</span>` : ''}</button>`;
  }).join('');
  pickBoard.querySelectorAll('[data-cell]').forEach(button => {
    button.addEventListener('click', () => {
      const key = button.dataset.cell;
      if (state.pickMode === 'from') {
        pickSource.value = key;
        pickStatus.textContent = `From selected: ${button.querySelector('strong').textContent}`;
        setPickMode('to');
      } else {
        pickTarget.value = key;
        pickStatus.textContent = `To selected: ${button.querySelector('strong').textContent}`;
      }
      renderPickBoard();
    });
  });
}
function loadPickPoints() {
  api('/api/pick_locations')
    .then(data => {
      fillPickSelect(pickSource, data.sources || []);
      fillPickSelect(pickTarget, data.targets || []);
      renderPickBoard();
      pickStatus.textContent = `${(data.sources || []).length} sources, ${(data.targets || []).length} targets loaded.`;
      updateConnection(true);
    })
    .catch(err => {
      updateConnection(false);
      pickStatus.textContent = `Load failed: ${err.message}`;
      log(`Pick point load failed: ${err.message}`);
    });
}
function runPickPlace() {
  const btn = document.getElementById('pickPlaceBtn');
  const source = pickSource.value;
  const target = pickTarget.value;
  if (!source || !target) {
    pickStatus.textContent = 'Select both source and target.';
    return;
  }
  if (source === target) {
    pickStatus.textContent = 'Source and target must be different.';
    return;
  }
  btn.disabled = true;
  state.sending = false;
  document.getElementById('stopBtn').textContent = 'Resume send';
  pickStatus.textContent = `Moving ${source} -> ${target} ...`;
  log(`Pick/place start: ${source} -> ${target}`);
  api('/api/pick_place', { method: 'POST', body: JSON.stringify({ source, target }) })
    .then(data => {
      if (data.angles) data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      pickStatus.textContent = `Done: ${data.source.label} -> ${data.target.label}`;
      log(`Pick/place done: ${data.source.label} -> ${data.target.label}`);
    })
    .catch(err => {
      updateConnection(false);
      pickStatus.textContent = `Move failed: ${err.message}`;
      log(`Pick/place failed: ${err.message}`);
    })
    .finally(() => {
      btn.disabled = false;
    });
}
function runCenterGrabTest() {
  const btn = document.getElementById('pickCenterBtn');
  btn.disabled = true;
  state.sending = false;
  document.getElementById('stopBtn').textContent = 'Resume send';
  pickStatus.textContent = 'Running center grab test ...';
  log('Center grab test start');
  api('/api/center_grab_test', { method: 'POST', body: '{}' })
    .then(data => {
      if (data.angles) data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      pickStatus.textContent = 'Center block should now be lifted and held.';
      log('Center grab test done');
    })
    .catch(err => {
      updateConnection(false);
      pickStatus.textContent = `Center grab failed: ${err.message}`;
      log(`Center grab failed: ${err.message}`);
    })
    .finally(() => {
      btn.disabled = false;
    });
}
function runCenterPlaceTest() {
  const btn = document.getElementById('placeCenterBtn');
  btn.disabled = true;
  state.sending = false;
  document.getElementById('stopBtn').textContent = 'Resume send';
  pickStatus.textContent = 'Running center place test ...';
  log('Center place test start');
  api('/api/center_place_test', { method: 'POST', body: '{}' })
    .then(data => {
      if (data.angles) data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      pickStatus.textContent = 'Center object has been released.';
      log('Center place test done');
    })
    .catch(err => {
      updateConnection(false);
      pickStatus.textContent = `Center place failed: ${err.message}`;
      log(`Center place failed: ${err.message}`);
    })
    .finally(() => {
      btn.disabled = false;
    });
}
function runCenterPickPlaceCycle() {
  const btn = document.getElementById('cycleCenterBtn');
  btn.disabled = true;
  state.sending = false;
  document.getElementById('stopBtn').textContent = 'Resume send';
  pickStatus.textContent = 'Running center pick+place ...';
  log('Center pick+place start');
  api('/api/center_pick_place', { method: 'POST', body: '{}' })
    .then(data => {
      if (data.angles) data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      pickStatus.textContent = 'Center pick+place completed.';
      log('Center pick+place done');
    })
    .catch(err => {
      updateConnection(false);
      pickStatus.textContent = `Center pick+place failed: ${err.message}`;
      log(`Center pick+place failed: ${err.message}`);
    })
    .finally(() => {
      btn.disabled = false;
    });
}
function runFixedPickPlace(source, target, label) {
  const btnId = source === 'cell_21' ? 'midLeftToRightBtn' : 'midRightToLeftBtn';
  const btn = document.getElementById(btnId);
  btn.disabled = true;
  state.sending = false;
  document.getElementById('stopBtn').textContent = 'Resume send';
  pickStatus.textContent = `Running ${label} ...`;
  log(`${label} start`);
  api('/api/pick_place', { method: 'POST', body: JSON.stringify({ source, target }) })
    .then(data => {
      if (data.angles) data.angles.slice(0, 6).forEach((angle, i) => setAngle(i, angle, false));
      updateConnection(true);
      pickStatus.textContent = `${label} completed.`;
      log(`${label} done`);
    })
    .catch(err => {
      updateConnection(false);
      pickStatus.textContent = `${label} failed: ${err.message}`;
      log(`${label} failed: ${err.message}`);
    })
    .finally(() => {
      btn.disabled = false;
    });
}
document.getElementById('readBtn').addEventListener('click', readAngles);
document.getElementById('homeBtn').addEventListener('click', home);
document.getElementById('danceBtn').addEventListener('click', dance);
document.getElementById('boardViewBtn').addEventListener('click', findBoardView);
document.getElementById('scanBoardBtn').addEventListener('click', scanBoard);
document.getElementById('pickPlaceBtn').addEventListener('click', runPickPlace);
document.getElementById('pickCenterBtn').addEventListener('click', runCenterGrabTest);
document.getElementById('placeCenterBtn').addEventListener('click', runCenterPlaceTest);
document.getElementById('cycleCenterBtn').addEventListener('click', runCenterPickPlaceCycle);
document.getElementById('midLeftToRightBtn').addEventListener('click', () => runFixedPickPlace('cell_21', 'cell_23', '中左 -> 中右'));
document.getElementById('midRightToLeftBtn').addEventListener('click', () => runFixedPickPlace('cell_23', 'cell_21', '中右 -> 中左'));
document.getElementById('pickRefreshBtn').addEventListener('click', loadPickPoints);
track2PrepareStartBtn.addEventListener('click', () => runTrack2Action('/api/track2_prepare_start', '到起点'));
track2StartBtn.addEventListener('click', () => runTrack2Action('/api/track2_start', '开始下棋'));
track2NextBtn.addEventListener('click', () => runTrack2Action('/api/track2_next', '下一步'));
track2ResetBtn.addEventListener('click', () => runTrack2Action('/api/track2_reset', '重置棋局'));
document.getElementById('manualFillBtn').addEventListener('click', () => fillManualFromAngles(state.angles));
document.getElementById('manualConfirmBtn').addEventListener('click', confirmManualPose);
document.getElementById('manualImportBtn').addEventListener('click', importManualPoseText);
document.getElementById('manualExportBtn').addEventListener('click', exportManualPoseText);
manualRunBtn.addEventListener('click', runManualPose);
document.getElementById('paramFillDefaultBtn').addEventListener('click', fillParamPickPlaceDefaults);
document.getElementById('paramPickImportBtn').addEventListener('click', () => importPose5Text(paramPickText, paramPickInputs, '抓取', 'pick'));
document.getElementById('paramPickExportBtn').addEventListener('click', () => exportPose5Text(paramPickText, paramPickInputs, '抓取'));
document.getElementById('paramPlaceImportBtn').addEventListener('click', () => importPose5Text(paramPlaceText, paramPlaceInputs, '放置', 'place'));
document.getElementById('paramPlaceExportBtn').addEventListener('click', () => exportPose5Text(paramPlaceText, paramPlaceInputs, '放置'));
paramRunBtn.addEventListener('click', runParamPickPlace);
pickModeFrom.addEventListener('click', () => setPickMode('from'));
pickModeTo.addEventListener('click', () => setPickMode('to'));
pickSource.addEventListener('change', renderPickBoard);
pickTarget.addEventListener('change', renderPickBoard);
document.getElementById('stopBtn').addEventListener('click', () => { state.sending = !state.sending; document.getElementById('stopBtn').textContent = state.sending ? '停止发送' : '恢复发送'; log(state.sending ? '已恢复实时发送' : '已暂停实时发送'); });
cameraFeed.addEventListener('load', () => { cameraState.textContent = '实时'; });
cameraFeed.addEventListener('error', () => {
  cameraState.textContent = '断开，重试中';
  window.setTimeout(() => {
    cameraFeed.src = `/video_feed?t=${Date.now()}`;
  }, 1200);
});
renderServos();
renderManualPoseInputs();
renderParamPickPlaceInputs();
renderParamCellPresets();
renderBoardScan([[{color:'无'}, {color:'无'}, {color:'无'}], [{color:'无'}, {color:'无'}, {color:'无'}], [{color:'无'}, {color:'无'}, {color:'无'}]]);
renderTrack2Monitor([[{color:'无', owner:'空'}, {color:'无', owner:'空'}, {color:'无', owner:'空'}], [{color:'无', owner:'空'}, {color:'无', owner:'空'}, {color:'无', owner:'空'}], [{color:'无', owner:'空'}, {color:'无', owner:'空'}, {color:'无', owner:'空'}]], {own: [], opponent: []}, {move_index: 0});
loadPickPoints();
loadTrack2State();
window.setInterval(loadTrack2State, 1500);
readAngles();
log('页面已就绪');
</script>
</body>
</html>'''

def clamp_angles(values):
    if not isinstance(values, list) or len(values) != 6:
        raise ValueError("angles must be a list of 6 numbers")
    result = []
    for i, value in enumerate(values):
        try:
            angle = int(round(float(value)))
        except Exception:
            angle = HOME[i]
        angle = max(SERVO_MIN[i], min(SERVO_MAX[i], angle))
        result.append(angle)
    return result

def read_actual_angles():
    angles = []
    for servo_id in range(1, 7):
        try:
            angle = Arm.Arm_serial_servo_read(servo_id)
        except Exception:
            angle = None
        if angle is None or angle < 0:
            angle = current_angles[servo_id - 1]
        angles.append(int(angle))
    return clamp_angles(angles)

def _move_to_unlocked(angles, move_ms):
    global current_angles
    angles = clamp_angles(angles)
    move_ms = int(max(100, min(3000, move_ms)))
    Arm.Arm_serial_servo_write6(angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], move_ms)
    current_angles = angles[:]
    return angles

def move_to(angles, move_ms):
    with arm_lock:
        return _move_to_unlocked(angles, move_ms)

def pick_pose_to_angles(pose5, gripper=None):
    if not isinstance(pose5, list) or len(pose5) != 5:
        raise ValueError("pick pose must be a list of 5 numbers")
    if gripper is None:
        gripper = current_angles[5]
    return clamp_angles(list(pose5) + [gripper])

def _set_gripper_unlocked(angle, move_ms=400):
    global current_angles
    angle = int(max(SERVO_MIN[5], min(SERVO_MAX[5], round(float(angle)))))
    Arm.Arm_serial_servo_write(6, angle, int(move_ms))
    current_angles[5] = angle
    time.sleep(move_ms / 1000.0 + 0.08)
    return angle

def _move_pick_pose_unlocked(pose5, move_ms=PICK_MOVE_MS, gripper=None):
    angles = pick_pose_to_angles(pose5, gripper)
    _move_to_unlocked(angles, move_ms)
    time.sleep(move_ms / 1000.0 + PICK_SETTLE_SEC)
    return angles

def _move_pick_pose_array_unlocked(pose5, move_ms=PICK_MOVE_MS, gripper=None):
    global current_angles
    angles = pick_pose_to_angles(pose5, gripper)
    move_ms = int(max(100, min(3000, move_ms)))
    Arm.Arm_serial_servo_write6_array(angles, move_ms)
    current_angles = angles[:]
    time.sleep(move_ms / 1000.0 + PICK_SETTLE_SEC)
    return angles

def clamp_pick_pose5(values):
    return pick_pose_to_angles(values, GRIPPER_OPEN)[:5]

def board_aligned_pick_pose5(pose5):
    pose = clamp_pick_pose5(pose5[:])
    if not PICK_BOARD_ALIGN_ENABLED:
        return pose
    base = pose[0]
    wrist = PICK_BOARD_ALIGN_REFERENCE_WRIST - (base - PICK_BOARD_ALIGN_REFERENCE_BASE)
    wrist = max(PICK_BOARD_ALIGN_WRIST_MIN, min(PICK_BOARD_ALIGN_WRIST_MAX, wrist))
    pose[4] = int(round(max(SERVO_MIN[4], min(SERVO_MAX[4], wrist))))
    return clamp_pick_pose5(pose)

def board_cell_place_pose5(key):
    fixed = FIXED_BOARD_CELL_POSES.get(key)
    if fixed:
        return clamp_pick_pose5(fixed["place"][:])
    return clamp_pick_pose5(PICK_LOCATIONS[key]["pose"][:])

def board_cell_pick_pose5(key):
    fixed = FIXED_BOARD_CELL_POSES.get(key)
    if fixed:
        return clamp_pick_pose5(fixed["pick"][:])
    return clamp_pick_pose5(PICK_LOCATIONS[key]["pose"][:])

def load_board_cell_poses():
    if not os.path.exists(BOARD_CELL_POSE_PATH):
        return False
    with open(BOARD_CELL_POSE_PATH, "r") as fh:
        data = json.load(fh)
    cells = data.get("cells", data)
    if not isinstance(cells, dict):
        return False
    for key in BOARD_CELL_KEYS:
        item = cells.get(key)
        if isinstance(item, dict):
            pose = item.get("pose")
            recorded_at = item.get("recorded_at") or item.get("updated_at")
        else:
            pose = item
            recorded_at = None
        if key in PICK_LOCATIONS and isinstance(pose, list) and len(pose) == 5:
            PICK_LOCATIONS[key]["pose"] = clamp_pick_pose5(pose)
            if recorded_at:
                PICK_LOCATIONS[key]["recorded_at"] = recorded_at
            if isinstance(item, dict):
                last_place_angles = item.get("last_place_angles")
                if isinstance(last_place_angles, list) and len(last_place_angles) == 6:
                    PICK_LOCATIONS[key]["last_place_angles"] = clamp_angles(last_place_angles)
                last_place_pose = item.get("last_place_pose")
                if isinstance(last_place_pose, list) and len(last_place_pose) == 5:
                    PICK_LOCATIONS[key]["last_place_pose"] = clamp_pick_pose5(last_place_pose)
                last_pick_pose = item.get("last_pick_pose")
                if isinstance(last_pick_pose, list) and len(last_pick_pose) == 5:
                    pick_pose = clamp_pick_pose5(last_pick_pose)
                    PICK_LOCATIONS[key]["last_pick_pose"] = pick_pose
                    PICK_LOCATIONS[key]["pose"] = pick_pose
                last_pick_angles = item.get("last_pick_angles")
                if isinstance(last_pick_angles, list) and len(last_pick_angles) == 6:
                    PICK_LOCATIONS[key]["last_pick_angles"] = clamp_angles(last_pick_angles)
                pick_recorded_at = item.get("pick_recorded_at")
                if pick_recorded_at:
                    PICK_LOCATIONS[key]["pick_recorded_at"] = pick_recorded_at
    apply_fixed_board_cell_poses()
    return True

def save_board_cell_poses():
    apply_fixed_board_cell_poses()
    cells = {}
    for key in BOARD_CELL_KEYS:
        if key in PICK_LOCATIONS:
            cells[key] = {
                "label": PICK_LOCATIONS[key]["label"],
                "pose": PICK_LOCATIONS[key]["pose"],
                "last_place_pose": PICK_LOCATIONS[key].get("last_place_pose"),
                "last_place_angles": PICK_LOCATIONS[key].get("last_place_angles"),
                "recorded_at": PICK_LOCATIONS[key].get("recorded_at"),
                "last_pick_pose": PICK_LOCATIONS[key].get("last_pick_pose"),
                "last_pick_angles": PICK_LOCATIONS[key].get("last_pick_angles"),
                "pick_recorded_at": PICK_LOCATIONS[key].get("pick_recorded_at"),
            }
    payload = {"updated_at": time.strftime("%Y-%m-%d %H:%M:%S"), "cells": cells}
    tmp_path = BOARD_CELL_POSE_PATH + ".tmp"
    with open(tmp_path, "w") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    os.replace(tmp_path, BOARD_CELL_POSE_PATH)

def record_board_cell_pose(key, pose5, place_angles6=None):
    if key not in BOARD_CELL_KEYS or key not in PICK_LOCATIONS:
        return None
    pose = clamp_pick_pose5(pose5)
    PICK_LOCATIONS[key]["last_place_pose"] = pose
    if key not in FIXED_BOARD_CELL_POSES and not PICK_LOCATIONS[key].get("last_pick_pose"):
        PICK_LOCATIONS[key]["pose"] = pose
    if isinstance(place_angles6, list) and len(place_angles6) == 6:
        PICK_LOCATIONS[key]["last_place_angles"] = clamp_angles(place_angles6)
    else:
        PICK_LOCATIONS[key]["last_place_angles"] = pick_pose_to_angles(pose, GRIPPER_OPEN)
    PICK_LOCATIONS[key]["recorded_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    save_board_cell_poses()
    return pick_location_payload(key)

def record_board_cell_pick_pose(key, pose5, pick_angles6=None):
    if key not in BOARD_CELL_KEYS or key not in PICK_LOCATIONS:
        return None
    pose = clamp_pick_pose5(pose5)
    if key not in FIXED_BOARD_CELL_POSES:
        PICK_LOCATIONS[key]["pose"] = pose
    PICK_LOCATIONS[key]["last_pick_pose"] = pose
    if isinstance(pick_angles6, list) and len(pick_angles6) == 6:
        PICK_LOCATIONS[key]["last_pick_angles"] = clamp_angles(pick_angles6)
    else:
        PICK_LOCATIONS[key]["last_pick_angles"] = pick_pose_to_angles(pose, GRIPPER_CLOSE)
    PICK_LOCATIONS[key]["pick_recorded_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    save_board_cell_poses()
    return pick_location_payload(key)

def pick_location_payload(key):
    item = PICK_LOCATIONS[key]
    payload = {"key": key, "label": item["label"], "pose": item["pose"]}
    if item.get("fixed_pick_pose"):
        payload["fixed_pick_pose"] = item["fixed_pick_pose"]
    if item.get("fixed_place_pose"):
        payload["fixed_place_pose"] = item["fixed_place_pose"]
    if item.get("last_place_pose"):
        payload["last_place_pose"] = item["last_place_pose"]
    if item.get("last_place_angles"):
        payload["last_place_angles"] = item["last_place_angles"]
    if item.get("recorded_at"):
        payload["recorded_at"] = item["recorded_at"]
    if item.get("last_pick_pose"):
        payload["last_pick_pose"] = item["last_pick_pose"]
    if item.get("last_pick_angles"):
        payload["last_pick_angles"] = item["last_pick_angles"]
    if item.get("pick_recorded_at"):
        payload["pick_recorded_at"] = item["pick_recorded_at"]
    return payload

def list_pick_locations():
    sources = ["start_pose"] + BOARD_CELL_KEYS + ["layer_4", "layer_3", "layer_2", "layer_1", "yellow", "red", "green", "blue", "blue_2"]
    targets = BOARD_CELL_KEYS + ["yellow", "red", "green", "blue", "blue_2"]
    return {
        "sources": [pick_location_payload(key) for key in sources if key in PICK_LOCATIONS],
        "targets": [pick_location_payload(key) for key in targets if key in PICK_LOCATIONS],
    }

def run_pick_place(source_key, target_key):
    if source_key not in PICK_LOCATIONS:
        raise ValueError("unknown source: %s" % source_key)
    if target_key not in PICK_LOCATIONS:
        raise ValueError("unknown target: %s" % target_key)
    if source_key == target_key:
        raise ValueError("source and target must be different")

    source = PICK_LOCATIONS[source_key]
    target = PICK_LOCATIONS[target_key]
    steps = []

    with arm_lock:
        source_pose_used = board_cell_pick_pose5(source_key)
        target_pose_original = board_cell_place_pose5(target_key)
        target_pose_used = board_aligned_pick_pose5(target_pose_original)
        _set_gripper_unlocked(GRIPPER_OPEN, 400)
        steps.append({"name": "open", "angles": current_angles[:]})
        if target_pose_used != target_pose_original:
            steps.append({"name": "target_board_alignment", "from": target_pose_original, "pose": target_pose_used})

        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_OPEN)
        steps.append({"name": "safe_before_pick", "angles": angles})

        angles = _move_pick_pose_array_unlocked(source_pose_used, PICK_MOVE_MS, GRIPPER_OPEN)
        steps.append({"name": "at_source", "angles": angles})

        source_pose_used, source_detection, wrist_delta = detect_pick_pose_adjustment_unlocked(source_pose_used)
        steps.append({"name": "source_angle_detection", "detection": source_detection})
        if not source_detection.get("found"):
            steps.append({"name": "source_detection_missing_using_fixed_pose", "detection": source_detection})
        elif wrist_delta:
            angles = _move_pick_pose_array_unlocked(source_pose_used, 450, GRIPPER_OPEN)
            steps.append({"name": "align_wrist_to_source", "angles": angles, "wrist_delta": wrist_delta})

        _set_gripper_unlocked(GRIPPER_CLOSE, 500)
        actual_pick_angles = read_actual_angles()
        steps.append({"name": "grip", "angles": current_angles[:], "actual_angles": actual_pick_angles})

        recorded_source_pick = record_board_cell_pick_pose(source_key, actual_pick_angles[:5], actual_pick_angles)
        if recorded_source_pick:
            steps.append({"name": "record_source_pick_pose", "source": recorded_source_pick})

        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_CLOSE)
        steps.append({"name": "lift_source", "angles": angles})

        safe_target_wrist_pose = PICK_SAFE_POSE_5[:]
        safe_target_wrist_pose[4] = target_pose_used[4]
        angles = _move_pick_pose_array_unlocked(safe_target_wrist_pose, 500, GRIPPER_CLOSE)
        steps.append({"name": "align_target_wrist_above_board", "angles": angles})

        angles = _move_pick_pose_array_unlocked(target_pose_used, PICK_MOVE_MS, GRIPPER_CLOSE)
        steps.append({"name": "at_target", "angles": angles})

        _set_gripper_unlocked(GRIPPER_OPEN, 500)
        steps.append({"name": "release", "angles": current_angles[:]})

        recorded_target = record_board_cell_pose(target_key, target_pose_used, current_angles[:])
        if recorded_target:
            steps.append({"name": "record_target_pose", "target": recorded_target})

        lift_pose = target_pose_used[:]
        lift_pose[2] = min(90, lift_pose[2] + 20)
        angles = _move_pick_pose_array_unlocked(lift_pose, 800, GRIPPER_OPEN)
        steps.append({"name": "lift_target", "angles": angles})

        angles = _move_pick_pose_array_unlocked(PICK_HOME_POSE_5, 900, GRIPPER_OPEN)
        steps.append({"name": "pick_home", "angles": angles})

    return {
        "status": "ok",
        "source": pick_location_payload(source_key),
        "target": pick_location_payload(target_key),
        "angles": current_angles[:],
        "steps": steps,
    }

TRACK2_CELL_KEYS = BOARD_CELL_KEYS[:]
TRACK2_COLOR_TO_SCAN = {
    TRACK2_EMPTY: "无",
    TRACK2_BLUE: "蓝色",
    TRACK2_YELLOW: "黄色",
}

def require_track2_ai():
    if TRACK2_IMPORT_ERROR is not None or Track2AI is None:
        raise RuntimeError("track2_game_ai.py not available: %r" % (TRACK2_IMPORT_ERROR,))

def track2_color_from_payload(value):
    text = str(value or "").strip().lower()
    if text in ("blue", "b", "1", "蓝", "蓝色"):
        return TRACK2_BLUE
    if text in ("yellow", "y", "2", "黄", "黄色"):
        return TRACK2_YELLOW
    raise ValueError("unknown track2 color: %r" % (value,))

def track2_color_label(color):
    if color == TRACK2_BLUE:
        return "蓝色"
    if color == TRACK2_YELLOW:
        return "黄色"
    return "无"

def track2_config_from_payload(payload, reset=False):
    require_track2_ai()
    order = str(payload.get("order", TRACK2_STATE.get("order", "first"))).strip().lower()
    if order not in ("first", "second"):
        raise ValueError("order must be first or second")
    ai_color = track2_color_from_payload(payload.get("color", "blue"))
    first_player = ai_color if order == "first" else track2_other(ai_color)
    if reset or not TRACK2_STATE.get("configured"):
        TRACK2_STATE.update({
            "configured": True,
            "started": True,
            "order": order,
            "ai_color": ai_color,
            "first_player": first_player,
            "last_board": None,
            "move_index": 0,
            "last_move": None,
        })
    return {
        "order": TRACK2_STATE["order"],
        "ai_color": TRACK2_STATE["ai_color"],
        "first_player": TRACK2_STATE["first_player"],
    }

def track2_state_payload():
    board = TRACK2_STATE.get("last_board")
    return {
        "configured": TRACK2_STATE.get("configured", False),
        "started": TRACK2_STATE.get("started", False),
        "order": TRACK2_STATE.get("order"),
        "ai_color": track2_color_label(TRACK2_STATE.get("ai_color")),
        "first_player": track2_color_label(TRACK2_STATE.get("first_player")),
        "move_index": TRACK2_STATE.get("move_index", 0),
        "last_board": None if board is None else list(board),
        "last_board_text": None if board is None or track2_board_to_text is None else track2_board_to_text(board),
        "last_move": TRACK2_STATE.get("last_move"),
    }

def track2_display_board():
    board = TRACK2_STATE.get("last_board")
    if board is None:
        return tuple([TRACK2_EMPTY] * 9)
    return tuple(board)

def track2_cell_key(index):
    if index is None or index < 0 or index >= len(TRACK2_CELL_KEYS):
        raise ValueError("cell index out of range: %r" % (index,))
    return TRACK2_CELL_KEYS[index]

def track2_board_from_scan_cells(cells):
    board = []
    for row in cells:
        for cell in row:
            color = cell.get("color")
            if color == "蓝色":
                board.append(TRACK2_BLUE)
            elif color == "黄色":
                board.append(TRACK2_YELLOW)
            else:
                board.append(TRACK2_EMPTY)
    if len(board) != 9:
        raise ValueError("scan result must contain 9 cells")
    return tuple(board)

def track2_cells_from_board(board):
    ai_color = TRACK2_STATE.get("ai_color")
    opponent_color = None
    if track2_other is not None and ai_color in (TRACK2_BLUE, TRACK2_YELLOW):
        opponent_color = track2_other(ai_color)
    cells = []
    for row in range(3):
        result_row = []
        for col in range(3):
            idx = row * 3 + col
            value = board[idx]
            if value == ai_color:
                owner = "我方"
            elif value == opponent_color:
                owner = "对方"
            else:
                owner = "空"
            key = track2_cell_key(idx)
            result_row.append({
                "row": row + 1,
                "col": col + 1,
                "index": idx + 1,
                "key": key,
                "label": BOARD_CELL_LABELS.get(key, key),
                "color": TRACK2_COLOR_TO_SCAN.get(value, "无"),
                "owner": owner,
                "confidence": 1.0,
                "source": "track2_state",
            })
        cells.append(result_row)
    return cells

def track2_positions_from_board(board):
    ai_color = TRACK2_STATE.get("ai_color")
    opponent_color = track2_other(ai_color) if track2_other is not None and ai_color in (TRACK2_BLUE, TRACK2_YELLOW) else None
    own = []
    opponent = []
    for idx, value in enumerate(board):
        if value == TRACK2_EMPTY:
            continue
        key = track2_cell_key(idx)
        item = {
            "index": idx + 1,
            "key": key,
            "label": BOARD_CELL_LABELS.get(key, key),
            "color": track2_color_label(value),
        }
        if value == ai_color:
            own.append(item)
        elif value == opponent_color:
            opponent.append(item)
    return {"own": own, "opponent": opponent}

def track2_status_response():
    board = track2_display_board()
    return {
        "status": "ok",
        "state": track2_state_payload(),
        "cells": track2_cells_from_board(board),
        "positions": track2_positions_from_board(board),
        "ai_import_error": None if TRACK2_IMPORT_ERROR is None else repr(TRACK2_IMPORT_ERROR),
    }

def run_track2_reset(payload):
    order = str(payload.get("order", TRACK2_STATE.get("order", "first"))).strip().lower()
    if order not in ("first", "second"):
        order = "first"
    try:
        ai_color = track2_color_from_payload(payload.get("color", TRACK2_STATE.get("ai_color", TRACK2_BLUE)))
    except Exception:
        ai_color = TRACK2_BLUE
    if track2_other is not None:
        first_player = ai_color if order == "first" else track2_other(ai_color)
    else:
        first_player = ai_color
    with track2_lock:
        TRACK2_STATE.update({
            "configured": False,
            "started": False,
            "order": order,
            "ai_color": ai_color,
            "first_player": first_player,
            "last_board": tuple([TRACK2_EMPTY] * 9),
            "move_index": 0,
            "last_move": None,
        })
        response = track2_status_response()
    response["message"] = "棋局已重置为 0。"
    return response

def track2_move_payload(move):
    if move is None:
        return None
    return move.to_cells()

def move_track2_neutral():
    with arm_lock:
        return _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_OPEN)

def run_track2_prepare_start(payload):
    with track2_lock:
        track2_config_from_payload(payload, reset=True)
    with arm_lock:
        _set_gripper_unlocked(GRIPPER_OPEN, 400)
        safe_angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_OPEN)
        start_angles = _move_pick_pose_array_unlocked(PICK_START_POSE_5, PICK_MOVE_MS, GRIPPER_OPEN)
    return {
        "status": "ok",
        "message": "已到起点，请放置棋子后点击开始下棋。",
        "angles": start_angles,
        "safe_angles": safe_angles,
        "state": track2_state_payload(),
    }

def run_track2_turn_from_board(board, scan_result=None, skip_scan=False):
    require_track2_ai()
    ai_color = TRACK2_STATE["ai_color"]
    ai = Track2AI(ai_color=ai_color, first_player=TRACK2_STATE["first_player"], max_depth=8)
    observed = tuple(board)
    correction = None
    last_board = TRACK2_STATE.get("last_board")
    if last_board is not None:
        correction = track2_detect_tamper(last_board, observed, ai_color)

    if correction is not None:
        move = correction
        reason = "restore"
    else:
        if track2_winner(observed):
            return {
                "status": "ok",
                "message": "当前棋盘已有胜负，不再执行。",
                "board": list(observed),
                "board_text": track2_board_to_text(observed),
                "cells": track2_cells_from_board(observed),
                "winner": track2_color_label(track2_winner(observed)),
                "state": track2_state_payload(),
                "scan": scan_result,
                "skipped_scan": skip_scan,
            }
        move = ai.choose_move(observed, to_move=ai_color)
        reason = "ai"

    if move is None:
        return {
            "status": "ok",
            "message": "当前没有可执行动作。",
            "board": list(observed),
            "board_text": track2_board_to_text(observed),
            "cells": track2_cells_from_board(observed),
            "state": track2_state_payload(),
            "scan": scan_result,
            "skipped_scan": skip_scan,
        }

    source_key = "start_pose" if move.src is None else track2_cell_key(move.src)
    target_key = track2_cell_key(move.dst)
    arm_result = run_pick_place(source_key, target_key)
    neutral_angles = move_track2_neutral()
    board_after = track2_apply_move(observed, move)
    TRACK2_STATE["last_board"] = board_after
    TRACK2_STATE["move_index"] = int(TRACK2_STATE.get("move_index", 0)) + 1
    TRACK2_STATE["last_move"] = track2_move_payload(move)

    return {
        "status": "ok",
        "message": "已完成赛道2动作。",
        "reason": reason,
        "move": track2_move_payload(move),
        "source_key": source_key,
        "target_key": target_key,
        "board_before": list(observed),
        "board_before_text": track2_board_to_text(observed),
        "board_after": list(board_after),
        "board_after_text": track2_board_to_text(board_after),
        "cells": track2_cells_from_board(board_after),
        "winner": track2_color_label(track2_winner(board_after)) if track2_winner(board_after) else None,
        "arm": arm_result,
        "angles": neutral_angles,
        "state": track2_state_payload(),
        "scan": scan_result,
        "skipped_scan": skip_scan,
    }

def run_track2_start(payload):
    with track2_lock:
        config = track2_config_from_payload(payload, reset=True)
        if config["order"] == "first":
            empty_board = tuple([TRACK2_EMPTY] * 9)
            result = run_track2_turn_from_board(empty_board, scan_result=None, skip_scan=True)
            result["message"] = "先手首次已跳过扫描并完成落子。"
            return result
        scan_result = scan_board_state()
        board = track2_board_from_scan_cells(scan_result["cells"])
        return run_track2_turn_from_board(board, scan_result=scan_result, skip_scan=False)

def run_track2_next(payload):
    with track2_lock:
        if not TRACK2_STATE.get("configured"):
            track2_config_from_payload(payload, reset=True)
        scan_result = scan_board_state()
        board = track2_board_from_scan_cells(scan_result["cells"])
        return run_track2_turn_from_board(board, scan_result=scan_result, skip_scan=False)

def run_manual_pick_place(pick_pose5, place_pose5, pick_current=False):
    global current_angles
    pick_current = bool(pick_current)
    pick_pose = None if pick_current else clamp_pick_pose5(pick_pose5)
    place_pose = clamp_pick_pose5(place_pose5)
    steps = []

    with arm_lock:
        if pick_current:
            current_angles = read_actual_angles()
            pick_pose = current_angles[:5]
            steps.append({"name": "current_pick_pose", "pose": pick_pose, "angles": current_angles[:]})
            _set_gripper_unlocked(GRIPPER_CLOSE, 600)
            steps.append({"name": "grip_at_current", "pose": pick_pose, "angles": current_angles[:], "actual_angles": read_actual_angles()})
        else:
            angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_OPEN)
            steps.append({"name": "safe_open", "angles": angles})

            angles = _move_pick_pose_array_unlocked(pick_pose, PICK_MOVE_MS, GRIPPER_OPEN)
            steps.append({"name": "pick_open", "pose": pick_pose, "angles": angles})

            _set_gripper_unlocked(GRIPPER_CLOSE, 600)
            steps.append({"name": "grip_at_pick", "angles": current_angles[:], "actual_angles": read_actual_angles()})

        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_CLOSE)
        steps.append({"name": "lift_holding", "angles": angles})

        angles = _move_pick_pose_array_unlocked(place_pose, PICK_MOVE_MS, GRIPPER_CLOSE)
        steps.append({"name": "place_holding", "pose": place_pose, "angles": angles})

        _set_gripper_unlocked(GRIPPER_OPEN, 600)
        steps.append({"name": "release_at_place", "angles": current_angles[:], "actual_angles": read_actual_angles()})

        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_OPEN)
        steps.append({"name": "lift_open", "angles": angles})

    return {
        "status": "ok",
        "pick_current": pick_current,
        "pick_pose": pick_pose,
        "place_pose": place_pose,
        "angles": current_angles[:],
        "steps": steps,
    }

def run_center_grab_test():
    steps = []
    with arm_lock:
        center_pose = board_cell_pick_pose5("cell_22")
        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 1200, GRIPPER_OPEN)
        steps.append({"name": "safe_open", "angles": angles})

        _set_gripper_unlocked(GRIPPER_OPEN, 500)
        steps.append({"name": "open", "angles": current_angles[:]})

        angles = _move_pick_pose_array_unlocked(center_pose, 1000, GRIPPER_OPEN)
        steps.append({"name": "at_center", "pose": center_pose, "angles": angles})

        _set_gripper_unlocked(GRIPPER_CLOSE, 600)
        actual_pick_angles = read_actual_angles()
        steps.append({"name": "grip_center", "angles": current_angles[:], "actual_angles": actual_pick_angles})

        recorded_source_pick = record_board_cell_pick_pose("cell_22", actual_pick_angles[:5], actual_pick_angles)
        if recorded_source_pick:
            steps.append({"name": "record_center_pick_pose", "source": recorded_source_pick})

        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 1200, GRIPPER_CLOSE)
        steps.append({"name": "lift_center", "angles": angles})

    return {
        "status": "ok",
        "source": pick_location_payload("cell_22"),
        "angles": current_angles[:],
        "steps": steps,
    }

def run_center_place_test():
    steps = []
    with arm_lock:
        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_CLOSE)
        steps.append({"name": "safe_holding", "angles": angles})

        angles = _move_pick_pose_array_unlocked(PICK_CENTER_POSE_5, 1000, GRIPPER_CLOSE)
        steps.append({"name": "at_center_holding", "angles": angles})

        _set_gripper_unlocked(GRIPPER_OPEN, 600)
        steps.append({"name": "release_center", "angles": current_angles[:]})

        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 1200, GRIPPER_OPEN)
        steps.append({"name": "lift_after_release", "angles": angles})

    return {
        "status": "ok",
        "target": pick_location_payload("cell_22"),
        "angles": current_angles[:],
        "steps": steps,
    }

def run_center_pick_place_cycle():
    steps = []
    with arm_lock:
        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 1200, GRIPPER_OPEN)
        steps.append({"name": "safe_open", "angles": angles})

        _set_gripper_unlocked(GRIPPER_OPEN, 500)
        steps.append({"name": "open", "angles": current_angles[:]})

        angles = _move_pick_pose_array_unlocked(PICK_CENTER_POSE_5, 1000, GRIPPER_OPEN)
        steps.append({"name": "at_center_pick", "angles": angles})

        _set_gripper_unlocked(GRIPPER_CLOSE, 600)
        steps.append({"name": "grip_center", "angles": current_angles[:]})

        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 1200, GRIPPER_CLOSE)
        steps.append({"name": "lift_holding", "angles": angles})

        angles = _move_pick_pose_array_unlocked(PICK_CENTER_POSE_5, 1000, GRIPPER_CLOSE)
        steps.append({"name": "at_center_place", "angles": angles})

        _set_gripper_unlocked(GRIPPER_OPEN, 600)
        steps.append({"name": "release_center", "angles": current_angles[:]})

        angles = _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 1200, GRIPPER_OPEN)
        steps.append({"name": "lift_after_release", "angles": angles})

    return {
        "status": "ok",
        "source": pick_location_payload("cell_22"),
        "target": pick_location_payload("cell_22"),
        "angles": current_angles[:],
        "steps": steps,
    }

def _chirp_unlocked(delay=1, rest=0.08):
    Arm.Arm_Buzzer_On(delay)
    time.sleep(rest)
    Arm.Arm_Buzzer_Off()

def run_dance():
    with arm_lock:
        try:
            Arm.Arm_RGB_set(0, 0, 0)
            _move_to_unlocked(HOME, 700)
            _chirp_unlocked(1, 0.12)
            for _ in range(2):
                for angles, color in DANCE_POSES:
                    Arm.Arm_RGB_set(color[0], color[1], color[2])
                    _chirp_unlocked(1, 0.06)
                    _move_to_unlocked(angles, 650)
                    time.sleep(0.1)
            for color in [(255, 255, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255)]:
                Arm.Arm_RGB_set(color[0], color[1], color[2])
                _chirp_unlocked(1, 0.05)
                time.sleep(0.1)
        finally:
            angles = _move_to_unlocked(HOME, 800)
            Arm.Arm_Buzzer_Off()
            Arm.Arm_RGB_set(0, 0, 0)
        return angles

def save_board_view_pose(angles, board_result):
    payload = {
        "board_view_pose": angles,
        "board": board_result,
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    tmp_path = CALIBRATION_PATH + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
    os.replace(tmp_path, CALIBRATION_PATH)
    return payload

def import_cv2():
    global cv2_module
    if cv2_module is not None:
        return cv2_module
    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("OpenCV import failed: %r" % (exc,))
    cv2_module = cv2
    return cv2

def open_camera():
    cv2 = import_cv2()
    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError("camera %d is not available" % CAMERA_ID)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS)
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    except Exception:
        pass
    return cap, cv2

def camera_worker():
    global latest_frame, latest_frame_time, latest_frame_error
    cap = None
    cv2 = None
    while not camera_stop_event.is_set():
        try:
            if cap is None:
                cap, cv2 = open_camera()
                latest_frame_error = None
            ok, frame = cap.read()
            if ok and frame is not None:
                with camera_lock:
                    latest_frame = frame.copy()
                    latest_frame_time = time.time()
                    latest_frame_error = None
            else:
                latest_frame_error = "camera read failed"
                time.sleep(0.05)
        except Exception as exc:
            latest_frame_error = repr(exc)
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
            time.sleep(1.0)
    if cap is not None:
        cap.release()

def start_camera_thread():
    global camera_thread
    if camera_thread is not None and camera_thread.is_alive():
        return
    camera_stop_event.clear()
    camera_thread = threading.Thread(target=camera_worker, name="camera-worker", daemon=True)
    camera_thread.start()

def get_latest_frame(timeout=3.0):
    start_camera_thread()
    deadline = time.time() + timeout
    while time.time() < deadline:
        with camera_lock:
            if latest_frame is not None and time.time() - latest_frame_time < 2.0:
                return latest_frame.copy()
            error = latest_frame_error
        time.sleep(0.05)
    raise RuntimeError("no camera frame available; last error=%r" % (error,))

def get_fresh_frame(after_time, timeout=3.0):
    start_camera_thread()
    deadline = time.time() + timeout
    error = None
    while time.time() < deadline:
        with camera_lock:
            if latest_frame is not None and latest_frame_time >= after_time:
                return latest_frame.copy()
            error = latest_frame_error
        time.sleep(0.03)
    return get_latest_frame(timeout=0.8)

def median_line_position(lines, axis):
    if not lines:
        return None
    values = []
    for x1, y1, x2, y2, length in lines:
        values.append((x1 + x2) / 2.0 if axis == "x" else (y1 + y2) / 2.0)
    values.sort()
    return values[len(values) // 2]

def cluster_positions(values, tolerance):
    clusters = []
    for value in sorted(values):
        placed = False
        for cluster in clusters:
            if abs(value - cluster["center"]) <= tolerance:
                cluster["values"].append(value)
                cluster["center"] = sum(cluster["values"]) / len(cluster["values"])
                placed = True
                break
        if not placed:
            clusters.append({"center": float(value), "values": [value]})
    return [cluster["center"] for cluster in clusters]

def choose_grid_by_span(positions, min_span, max_span):
    positions = sorted(float(v) for v in positions)
    if len(positions) < 4:
        return None
    best = None
    for i in range(len(positions) - 3):
        for j in range(i + 1, len(positions) - 2):
            for k in range(j + 1, len(positions) - 1):
                for m in range(k + 1, len(positions)):
                    grid = [positions[i], positions[j], positions[k], positions[m]]
                    gaps = [grid[1] - grid[0], grid[2] - grid[1], grid[3] - grid[2]]
                    span = grid[3] - grid[0]
                    if span < min_span or span > max_span:
                        continue
                    if min(gaps) <= 0:
                        continue
                    avg_gap = sum(gaps) / 3.0
                    gap_error = max(abs(gap - avg_gap) for gap in gaps) / avg_gap
                    score = span - gap_error * 120.0
                    if best is None or score > best["score"]:
                        best = {
                            "lines": grid,
                            "gaps": gaps,
                            "span": span,
                            "gap_error": gap_error,
                            "score": score,
                        }
    return best

def line_support(mask, horizontal_grid, vertical_grid):
    cv2 = import_cv2()
    np = __import__("numpy")
    height, width = mask.shape[:2]
    x1 = int(max(0, min(vertical_grid)))
    x2 = int(min(width - 1, max(vertical_grid)))
    y1 = int(max(0, min(horizontal_grid)))
    y2 = int(min(height - 1, max(horizontal_grid)))
    if x2 <= x1 or y2 <= y1:
        return {
            "horizontal": [],
            "vertical": [],
            "mean": 0.0,
            "intersections": 0,
        }

    horizontal_scores = []
    for y in horizontal_grid:
        yy = int(round(y))
        top = max(0, yy - 5)
        bottom = min(height, yy + 6)
        band = mask[top:bottom, x1:x2]
        horizontal_scores.append(float(cv2.countNonZero(band)) / float(max(1, band.size)))

    vertical_scores = []
    for x in vertical_grid:
        xx = int(round(x))
        left = max(0, xx - 5)
        right = min(width, xx + 6)
        band = mask[y1:y2, left:right]
        vertical_scores.append(float(cv2.countNonZero(band)) / float(max(1, band.size)))

    intersections = 0
    for y in horizontal_grid:
        yy = int(round(y))
        for x in vertical_grid:
            xx = int(round(x))
            patch = mask[max(0, yy - 8):min(height, yy + 9), max(0, xx - 8):min(width, xx + 9)]
            if patch.size and float(cv2.countNonZero(patch)) / float(patch.size) > 0.10:
                intersections += 1

    all_scores = horizontal_scores + vertical_scores
    return {
        "horizontal": [round(v, 3) for v in horizontal_scores],
        "vertical": [round(v, 3) for v in vertical_scores],
        "mean": float(sum(all_scores) / len(all_scores)) if all_scores else 0.0,
        "intersections": intersections,
    }

def choose_intersection_grid(horizontal_positions, vertical_positions, mask):
    cv2 = import_cv2()
    height, width = mask.shape[:2]
    h_values = sorted(float(v) for v in horizontal_positions if height * 0.03 <= float(v) <= height * 0.97)
    v_values = sorted(float(v) for v in vertical_positions if width * 0.02 <= float(v) <= width * 0.98)
    best = None

    def patch_support(x, y, radius=10):
        xx = int(round(x))
        yy = int(round(y))
        patch = mask[max(0, yy - radius):min(height, yy + radius + 1), max(0, xx - radius):min(width, xx + radius + 1)]
        if patch.size == 0:
            return 0.0
        return float(cv2.countNonZero(patch)) / float(patch.size)

    def band_support_x(x, y1, y2):
        xx = int(round(x))
        top = int(max(0, round(min(y1, y2))))
        bottom = int(min(height, round(max(y1, y2))))
        band = mask[top:bottom, max(0, xx - 5):min(width, xx + 6)]
        if band.size == 0:
            return 0.0
        return float(cv2.countNonZero(band)) / float(band.size)

    def band_support_y(y, x1, x2):
        yy = int(round(y))
        left = int(max(0, round(min(x1, x2))))
        right = int(min(width, round(max(x1, x2))))
        band = mask[max(0, yy - 5):min(height, yy + 6), left:right]
        if band.size == 0:
            return 0.0
        return float(cv2.countNonZero(band)) / float(band.size)

    for hi in range(len(h_values) - 1):
        for hj in range(hi + 1, len(h_values)):
            h_pair = [h_values[hi], h_values[hj]]
            cell_h = h_pair[1] - h_pair[0]
            if cell_h < height * 0.11 or cell_h > height * 0.46:
                continue
            for vi in range(len(v_values) - 1):
                for vj in range(vi + 1, len(v_values)):
                    v_pair = [v_values[vi], v_values[vj]]
                    cell_w = v_pair[1] - v_pair[0]
                    if cell_w < width * 0.11 or cell_w > width * 0.48:
                        continue
                    aspect = cell_w / cell_h if cell_h else 0.0
                    if aspect < 0.55 or aspect > 1.85:
                        continue

                    intersections = [patch_support(x, y) for y in h_pair for x in v_pair]
                    strong_intersections = sum(1 for value in intersections if value >= 0.08)
                    min_intersection = min(intersections) if intersections else 0.0
                    if strong_intersections < 3:
                        continue

                    h_support = [band_support_y(y, v_pair[0], v_pair[1]) for y in h_pair]
                    v_support = [band_support_x(x, h_pair[0], h_pair[1]) for x in v_pair]
                    support_mean = sum(intersections + h_support + v_support) / float(len(intersections) + len(h_support) + len(v_support))

                    expected_h = [h_pair[0] - cell_h, h_pair[0], h_pair[1], h_pair[1] + cell_h]
                    expected_v = [v_pair[0] - cell_w, v_pair[0], v_pair[1], v_pair[1] + cell_w]
                    frame_h = [0.0, h_pair[0], h_pair[1], float(height)]
                    frame_v = [0.0, v_pair[0], v_pair[1], float(width)]
                    visible_h = 2
                    visible_v = 2
                    center_cell_area = cell_w * cell_h
                    center_cell_area_ratio = center_cell_area / float(width * height)
                    center_x = (v_pair[0] + v_pair[1]) / 2.0
                    center_y = (h_pair[0] + h_pair[1]) / 2.0
                    center_error = abs(center_x - width / 2.0) / width + abs(center_y - height / 2.0) / height

                    frame_cells = []
                    for row in range(3):
                        for col in range(3):
                            x0 = frame_v[col]
                            x1 = frame_v[col + 1]
                            y0 = frame_h[row]
                            y1 = frame_h[row + 1]
                            frame_cells.append(max(0.0, x1 - x0) * max(0.0, y1 - y0))
                    min_cell = min(frame_cells) if frame_cells else 0.0
                    max_cell = max(frame_cells) if frame_cells else 1.0
                    uniformity = min_cell / max(1.0, max_cell)
                    visible_cell_count = sum(1 for area in frame_cells if area >= max_cell * 0.25)

                    score = 0.0
                    score += strong_intersections * 130.0
                    score += support_mean * 520.0
                    score += visible_h * 35.0 + visible_v * 35.0
                    score += visible_cell_count * 22.0
                    score += max(0.0, 1.0 - abs(1.0 - aspect)) * 120.0
                    score += min(center_cell_area_ratio, 0.28) * 420.0
                    score += max(0.0, 1.0 - center_error * 2.0) * 90.0
                    score += uniformity * 260.0

                    if best is None or score > best["score"]:
                        best = {
                            "grid_style": "two_internal_lines",
                            "score": score,
                            "horizontal_pair": h_pair,
                            "vertical_pair": v_pair,
                            "cell_width": cell_w,
                            "cell_height": cell_h,
                            "aspect": aspect,
                            "support_mean": support_mean,
                            "min_intersection": min_intersection,
                            "strong_intersections": strong_intersections,
                            "visible_expected_lines": {"horizontal": visible_h, "vertical": visible_v},
                            "visible_cell_count": visible_cell_count,
                            "uniformity": uniformity,
                            "area_ratio": center_cell_area_ratio,
                            "center_error": center_error,
                            "expected_horizontal": expected_h,
                            "expected_vertical": expected_v,
                            "frame_rows": [0.0, h_pair[0], h_pair[1], float(height)],
                            "frame_cols": [0.0, v_pair[0], v_pair[1], float(width)],
                        }

    if best is None:
        return {
            "found": False,
            "grid_style": "none",
            "score": 0.0,
            "horizontal_pair": [],
            "vertical_pair": [],
            "expected_horizontal": [],
            "expected_vertical": [],
            "frame_rows": [],
            "frame_cols": [],
            "strong_intersections": 0,
            "support_mean": 0.0,
            "min_intersection": 0.0,
            "uniformity": 0.0,
            "visible_cell_count": 0,
            "visible_expected_lines": {"horizontal": 0, "vertical": 0},
            "area_ratio": 0.0,
            "center_error": 1.0,
            "aspect": 0.0,
        }

    best["found"] = True
    best["score"] = round(best["score"], 2)
    best["horizontal_pair"] = [round(v, 1) for v in best["horizontal_pair"]]
    best["vertical_pair"] = [round(v, 1) for v in best["vertical_pair"]]
    best["expected_horizontal"] = [round(v, 1) for v in best["expected_horizontal"]]
    best["expected_vertical"] = [round(v, 1) for v in best["expected_vertical"]]
    best["frame_rows"] = [round(v, 1) for v in best["frame_rows"]]
    best["frame_cols"] = [round(v, 1) for v in best["frame_cols"]]
    best["cell_width"] = round(best["cell_width"], 1)
    best["cell_height"] = round(best["cell_height"], 1)
    best["aspect"] = round(best["aspect"], 4)
    best["support_mean"] = round(best["support_mean"], 4)
    best["min_intersection"] = round(best["min_intersection"], 4)
    best["uniformity"] = round(best["uniformity"], 4)
    best["area_ratio"] = round(best["area_ratio"], 4)
    best["center_error"] = round(best["center_error"], 4)
    return best

def analyze_board_frame(frame):
    cv2 = import_cv2()
    np = __import__("numpy")
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, dark_mask = cv2.threshold(blur, 85, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((3, 3), np.uint8)
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)
    edges = cv2.Canny(blur, 55, 150)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=62,
        minLineLength=max(55, int(min(width, height) * 0.12)),
        maxLineGap=22,
    )

    horizontal = []
    vertical = []
    if lines is not None:
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(v) for v in line]
            dx = x2 - x1
            dy = y2 - y1
            length = float((dx * dx + dy * dy) ** 0.5)
            if length < 55:
                continue
            angle = abs(np.degrees(np.arctan2(dy, dx)))
            if angle < 13 or angle > 167:
                horizontal.append((x1, y1, x2, y2, length))
            elif 77 < angle < 103:
                vertical.append((x1, y1, x2, y2, length))

    h_pos_all = cluster_positions([median_line_position([line], "y") for line in horizontal], 26)
    v_pos_all = cluster_positions([median_line_position([line], "x") for line in vertical], 26)

    intersection_grid = choose_intersection_grid(h_pos_all, v_pos_all, dark_mask)

    h_grid = choose_grid_by_span(h_pos_all, height * 0.32, height * 0.95)
    v_grid = choose_grid_by_span(v_pos_all, width * 0.32, width * 0.95)
    if h_grid is not None and v_grid is not None:
        h_pos = h_grid["lines"]
        v_pos = v_grid["lines"]
    else:
        h_pos = []
        v_pos = []

    board_w = max(v_pos) - min(v_pos) if len(v_pos) == 4 else 0.0
    board_h = max(h_pos) - min(h_pos) if len(h_pos) == 4 else 0.0
    margin_left = min(v_pos) if v_pos else 0.0
    margin_right = width - max(v_pos) if v_pos else 0.0
    margin_top = min(h_pos) if h_pos else 0.0
    margin_bottom = height - max(h_pos) if h_pos else 0.0

    center_x = (min(v_pos) + max(v_pos)) / 2.0 if len(v_pos) == 4 else width / 2.0
    center_y = (min(h_pos) + max(h_pos)) / 2.0 if len(h_pos) == 4 else height / 2.0
    center_error = abs(center_x - width / 2.0) / width + abs(center_y - height / 2.0) / height
    area_ratio = (board_w * board_h) / float(width * height) if board_w and board_h else 0.0
    aspect = board_w / board_h if board_h else 0.0
    support = line_support(dark_mask, h_pos, v_pos) if len(h_pos) == 4 and len(v_pos) == 4 else {
        "horizontal": [],
        "vertical": [],
        "mean": 0.0,
        "intersections": 0,
    }

    margin_ok = (
        margin_left >= 18 and margin_right >= 18 and
        margin_top >= 18 and margin_bottom >= 18
    )
    line_ok = len(h_pos) == 4 and len(v_pos) == 4
    size_ok = area_ratio >= 0.24
    aspect_ok = 0.72 <= aspect <= 1.35
    grid_ok = h_grid is not None and v_grid is not None
    support_ok = support["mean"] >= 0.08 and support["intersections"] >= 12
    stable_inner_grid = (
        intersection_grid["found"] and
        intersection_grid["strong_intersections"] >= 4 and
        intersection_grid["min_intersection"] >= 0.16 and
        intersection_grid["support_mean"] >= 0.14 and
        intersection_grid["uniformity"] >= 0.25 and
        intersection_grid["visible_cell_count"] >= 7 and
        0.07 <= intersection_grid["area_ratio"] <= 0.32 and
        intersection_grid["visible_expected_lines"]["horizontal"] >= 2 and
        intersection_grid["visible_expected_lines"]["vertical"] >= 2
    )
    complete = bool(
        (line_ok and margin_ok and size_ok and aspect_ok and grid_ok and support_ok) or
        stable_inner_grid
    )

    score = 0.0
    score += min(len(h_pos), 4) * 90
    score += min(len(v_pos), 4) * 90
    score += min(len(horizontal), 18) * 5
    score += min(len(vertical), 18) * 5
    score += min(area_ratio, 0.72) * 220
    score += max(0.0, 1.0 - center_error * 2.2) * 120
    score += support["mean"] * 220
    score += min(support["intersections"], 16) * 10
    score += intersection_grid["score"]
    score += intersection_grid["uniformity"] * 160
    score += intersection_grid["strong_intersections"] * 35
    score += intersection_grid["visible_cell_count"] * 12
    if grid_ok:
        score += 130
    else:
        score -= 240
    if aspect_ok:
        score += 80
    else:
        score -= 180
    if margin_ok:
        score += 90
    else:
        score -= 220
    if complete:
        score += 180
    if min(margin_left, margin_right, margin_top, margin_bottom) < 10:
        score -= 200

    return {
        "score": round(score, 2),
        "complete": complete,
        "horizontal_lines": len(horizontal),
        "vertical_lines": len(vertical),
        "horizontal_candidates": [round(v, 1) for v in h_pos_all],
        "vertical_candidates": [round(v, 1) for v in v_pos_all],
        "horizontal_clusters": [round(v, 1) for v in h_pos],
        "vertical_clusters": [round(v, 1) for v in v_pos],
        "area_ratio": round(area_ratio, 4),
        "aspect": round(aspect, 4),
        "center_error": round(center_error, 4),
        "regularity": {
            "horizontal_gap_error": None if h_grid is None else round(h_grid["gap_error"], 4),
            "vertical_gap_error": None if v_grid is None else round(v_grid["gap_error"], 4),
        },
        "intersection_grid": intersection_grid,
        "support": support,
        "margins": {
            "left": round(margin_left, 1),
            "right": round(margin_right, 1),
            "top": round(margin_top, 1),
            "bottom": round(margin_bottom, 1),
        },
    }

def classify_board_cell(frame, x1, y1, x2, y2):
    cv2 = import_cv2()
    np = __import__("numpy")
    height, width = frame.shape[:2]
    left = int(max(0, min(width - 1, round(x1))))
    right = int(max(left + 1, min(width, round(x2))))
    top = int(max(0, min(height - 1, round(y1))))
    bottom = int(max(top + 1, min(height, round(y2))))

    cell = frame[top:bottom, left:right]
    if cell.size == 0:
        return {"color": "无", "confidence": 0.0, "yellow_ratio": 0.0, "blue_ratio": 0.0}

    h, w = cell.shape[:2]
    pad_x = max(2, int(w * 0.18))
    pad_y = max(2, int(h * 0.18))
    roi = cell[pad_y:max(pad_y + 1, h - pad_y), pad_x:max(pad_x + 1, w - pad_x)]
    if roi.size == 0:
        roi = cell

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    masks = {
        "黄色": cv2.inRange(hsv, np.array([18, 90, 70]), np.array([42, 255, 255])),
        "蓝色": cv2.inRange(hsv, np.array([92, 95, 45]), np.array([128, 255, 245])),
        "黑色": cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 65])),
        "红色": cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 90, 60]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([170, 90, 60]), np.array([180, 255, 255])),
        ),
        "绿色": cv2.inRange(hsv, np.array([45, 70, 55]), np.array([88, 255, 255])),
    }
    kernel = np.ones((3, 3), np.uint8)
    for name in list(masks.keys()):
        masks[name] = cv2.morphologyEx(masks[name], cv2.MORPH_OPEN, kernel)

    total = float(max(1, roi.shape[0] * roi.shape[1]))
    ratios = {name: float(cv2.countNonZero(mask)) / total for name, mask in masks.items()}
    ranked = sorted(ratios.items(), key=lambda item: item[1], reverse=True)
    best_color, best_ratio = ranked[0]
    second_ratio = ranked[1][1] if len(ranked) > 1 else 0.0
    if best_ratio >= 0.12 and best_ratio >= second_ratio * 1.35:
        color = best_color
        confidence = best_ratio
    else:
        color = "无"
        confidence = best_ratio

    result = {
        "color": color,
        "confidence": round(confidence, 4),
    }
    for name, ratio in ratios.items():
        result[name + "_ratio"] = round(ratio, 4)
    return result

def active_scan_targets(center_pose):
    targets = []
    for target in FIXED_BOARD_SCAN_TARGETS:
        targets.append({
            "index": len(targets) + 1,
            "row": target["row"],
            "col": target["col"],
            "label": target["label"],
            "pose": clamp_angles(target["pose"]),
            "offset": [0, 0],
        })
    return targets

def central_sample_box(frame, pad=ACTIVE_SCAN_SAMPLE_PAD):
    height, width = frame.shape[:2]
    x1 = width * (0.5 - pad)
    x2 = width * (0.5 + pad)
    y1 = height * (0.5 - pad)
    y2 = height * (0.5 + pad)
    return [x1, y1, x2, y2]

def normalize_rect_angle(angle, rect_w, rect_h):
    angle = float(angle)
    if rect_w < rect_h:
        angle += 90.0
    while angle <= -45.0:
        angle += 90.0
    while angle > 45.0:
        angle -= 90.0
    return angle

def detect_block_angle_from_frame(frame):
    cv2 = import_cv2()
    np = __import__("numpy")
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = central_sample_box(frame, PICK_VISION_CROP_PAD)
    left = int(max(0, min(width - 1, round(x1))))
    right = int(max(left + 1, min(width, round(x2))))
    top = int(max(0, min(height - 1, round(y1))))
    bottom = int(max(top + 1, min(height, round(y2))))
    roi = frame[top:bottom, left:right]
    if roi.size == 0:
        return {"found": False, "reason": "empty roi"}

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array([18, 45, 45]), np.array([42, 255, 255]))
    blue = cv2.inRange(hsv, np.array([90, 45, 35]), np.array([130, 255, 255]))
    red = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 40, 40]), np.array([15, 255, 255])),
        cv2.inRange(hsv, np.array([160, 40, 40]), np.array([180, 255, 255])),
    )
    green = cv2.inRange(hsv, np.array([42, 40, 35]), np.array([90, 255, 255]))
    kernel = np.ones((5, 5), np.uint8)
    color_masks = {
        "yellow": yellow,
        "blue": blue,
        "red": red,
        "green": green,
    }
    roi_area = float(max(1, roi.shape[0] * roi.shape[1]))
    color_ratios = {}
    candidates = []
    center_x = roi.shape[1] / 2.0
    center_y = roi.shape[0] / 2.0
    max_dist = min(roi.shape[0], roi.shape[1]) * PICK_VISION_CENTER_TOLERANCE
    for color, mask in color_masks.items():
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        color_ratios[color] = round(float(cv2.countNonZero(mask)) / roi_area, 4)
        if color_ratios[color] > PICK_VISION_MAX_AREA_RATIO:
            continue
        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_result[-2]
        for contour in contours:
            area = float(cv2.contourArea(contour))
            area_ratio = area / roi_area
            if area_ratio < PICK_VISION_MIN_AREA_RATIO or area_ratio > PICK_VISION_MAX_AREA_RATIO:
                continue
            rect = cv2.minAreaRect(contour)
            (cx, cy), (rw, rh), raw_angle = rect
            if rw < 8 or rh < 8:
                continue
            dist = ((cx - center_x) ** 2 + (cy - center_y) ** 2) ** 0.5
            if dist > max_dist:
                continue
            angle = normalize_rect_angle(raw_angle, rw, rh)
            candidates.append({
                "color": color,
                "area": area,
                "area_ratio": area_ratio,
                "center_distance": dist,
                "angle": angle,
                "raw_angle": float(raw_angle),
                "rect": [round(cx + left, 1), round(cy + top, 1), round(rw, 1), round(rh, 1)],
            })

    if not candidates:
        return {
            "found": False,
            "reason": "no centered colored contour",
            "box": [round(left, 1), round(top, 1), round(right, 1), round(bottom, 1)],
            "color_ratios": color_ratios,
        }

    best = max(candidates, key=lambda item: (item["area"], -item["center_distance"]))
    return {
        "found": True,
        "angle": round(best["angle"], 2),
        "raw_angle": round(best["raw_angle"], 2),
        "color": best["color"],
        "area_ratio": round(best["area_ratio"], 4),
        "center_distance": round(best["center_distance"], 1),
        "rect": best["rect"],
        "box": [round(left, 1), round(top, 1), round(right, 1), round(bottom, 1)],
    }

def pose_with_detected_wrist_angle(pose5, detection):
    pose = pose5[:]
    if not detection.get("found"):
        return pose, 0
    angle = float(detection.get("angle", 0.0))
    delta = int(round(angle * PICK_VISION_WRIST_SIGN))
    delta = max(-PICK_VISION_WRIST_LIMIT, min(PICK_VISION_WRIST_LIMIT, delta))
    if pose[4] + delta > SERVO_MAX[4]:
        delta = -abs(int(round(angle)))
    elif pose[4] + delta < SERVO_MIN[4]:
        delta = abs(int(round(angle)))
    delta = max(-PICK_VISION_WRIST_LIMIT, min(PICK_VISION_WRIST_LIMIT, delta))
    pose[4] = int(max(SERVO_MIN[4], min(SERVO_MAX[4], pose[4] + delta)))
    return pose, delta

def pick_detection_center_error(detection):
    if not detection.get("found"):
        return None
    rect = detection.get("rect") or []
    box = detection.get("box") or []
    if len(rect) < 2 or len(box) != 4:
        return None
    try:
        cx = float(rect[0])
        cy = float(rect[1])
        center_x = (float(box[0]) + float(box[2])) / 2.0
        center_y = (float(box[1]) + float(box[3])) / 2.0
    except Exception:
        return None
    dx = cx - center_x
    dy = cy - center_y
    return {
        "dx": round(dx, 1),
        "dy": round(dy, 1),
        "distance": round((dx ** 2 + dy ** 2) ** 0.5, 1),
    }

def centered_pick_score(detection):
    if not detection.get("found"):
        return None
    center_error = pick_detection_center_error(detection)
    if center_error is None:
        return None
    area_bonus = float(detection.get("area_ratio", 0.0)) * 12.0
    return float(center_error["distance"]) - area_bonus

def search_centered_pick_pose_unlocked(base_pose5, first_detection):
    if not PICK_VISION_SEARCH_ENABLED:
        return base_pose5[:], first_detection

    first_detection["center_error"] = pick_detection_center_error(first_detection)
    if first_detection.get("found"):
        first_distance = float((first_detection.get("center_error") or {}).get("distance", first_detection.get("center_distance", 999.0)))
        if first_distance <= PICK_VISION_CENTER_OK_PIXELS:
            first_detection["center_search"] = {"used": False, "reason": "already_centered"}
            return base_pose5[:], first_detection
    else:
        first_distance = 999.0
        first_detection["center_search"] = {"used": True, "reason": "initial_detection_missing"}

    best_pose = base_pose5[:]
    best_detection = first_detection
    best_score = centered_pick_score(first_detection)
    candidates = []

    for base_delta, shoulder_delta in PICK_VISION_SEARCH_OFFSETS:
        candidate = base_pose5[:]
        candidate[0] += base_delta
        candidate[1] += shoulder_delta
        candidate = clamp_pick_pose5(candidate)
        if candidate == best_pose and candidates:
            continue
        moved_at = time.time()
        angles = _move_pick_pose_array_unlocked(candidate, PICK_VISION_SEARCH_MOVE_MS, GRIPPER_OPEN)
        time.sleep(PICK_VISION_SEARCH_SETTLE_SEC)
        try:
            frame = get_fresh_frame(moved_at, timeout=2.0)
            detection = detect_block_angle_from_frame(frame)
        except Exception as exc:
            detection = {"found": False, "reason": repr(exc)}
        detection["search_offset"] = [base_delta, shoulder_delta]
        detection["search_pose"] = candidate[:]
        detection["search_angles"] = angles
        detection["center_error"] = pick_detection_center_error(detection)
        score = centered_pick_score(detection)
        candidates.append({
            "offset": [base_delta, shoulder_delta],
            "pose": candidate[:],
            "found": detection.get("found"),
            "color": detection.get("color"),
            "area_ratio": detection.get("area_ratio"),
            "center_error": detection.get("center_error"),
            "score": None if score is None else round(score, 2),
            "reason": detection.get("reason"),
        })
        if score is not None and (best_score is None or score < best_score):
            best_score = score
            best_pose = candidate[:]
            best_detection = detection

    if best_pose != current_angles[:5]:
        _move_pick_pose_array_unlocked(best_pose, PICK_VISION_SEARCH_MOVE_MS, GRIPPER_OPEN)

    best_detection["center_search"] = {
        "used": best_pose != base_pose5,
        "start_distance": round(first_distance, 1),
        "best_score": None if best_score is None else round(best_score, 2),
        "candidates": candidates,
    }
    return best_pose, best_detection

def detect_pick_pose_adjustment_unlocked(pose5):
    if not PICK_VISION_ANGLE_ENABLED:
        return pose5[:], {"found": False, "reason": "disabled"}, 0
    frame = get_fresh_frame(time.time(), timeout=3.0)
    detection = detect_block_angle_from_frame(frame)
    detection["center_error"] = pick_detection_center_error(detection)
    pose_for_pick, detection = search_centered_pick_pose_unlocked(pose5, detection)
    adjusted_pose, delta = pose_with_detected_wrist_angle(pose_for_pick, detection)
    detection["wrist_delta"] = delta
    detection["adjusted_pose"] = adjusted_pose
    return adjusted_pose, detection, delta

def debug_detect_pick_angle(cell_key):
    if cell_key not in PICK_LOCATIONS:
        raise ValueError("unknown cell: %s" % cell_key)
    pose = PICK_LOCATIONS[cell_key]["pose"][:]
    with arm_lock:
        _set_gripper_unlocked(GRIPPER_OPEN, 400)
        _move_pick_pose_array_unlocked(PICK_SAFE_POSE_5, 900, GRIPPER_OPEN)
        angles = _move_pick_pose_array_unlocked(pose, PICK_MOVE_MS, GRIPPER_OPEN)
        adjusted_pose, detection, delta = detect_pick_pose_adjustment_unlocked(pose)
        adjusted_angles = None
        if delta:
            adjusted_angles = _move_pick_pose_array_unlocked(adjusted_pose, 450, GRIPPER_OPEN)
        _move_pick_pose_array_unlocked(PICK_HOME_POSE_5, 900, GRIPPER_OPEN)
    return {
        "status": "ok",
        "cell": pick_location_payload(cell_key),
        "at_cell_angles": angles,
        "detection": detection,
        "adjusted_angles": adjusted_angles,
    }

def choose_scan_sample(samples):
    votes = {}
    best_by_color = {}
    for sample in samples:
        color = sample.get("color", "无")
        confidence = float(sample.get("confidence", 0.0))
        if color == "无":
            continue
        votes[color] = votes.get(color, 0) + confidence
        if confidence > best_by_color.get(color, {}).get("confidence", -1):
            best_by_color[color] = sample

    if votes:
        best_color = max(votes.items(), key=lambda item: item[1])[0]
        best = dict(best_by_color[best_color])
        best["confidence"] = round(votes[best_color] / max(1, len(samples)), 4)
        best["frame_samples"] = samples
        best["vote_score"] = round(votes[best_color], 4)
        return best

    if samples:
        best = dict(max(samples, key=lambda item: float(item.get("confidence", 0.0))))
        best["frame_samples"] = samples
        best["vote_score"] = 0.0
        return best

    return {"color": "无", "confidence": 0.0, "frame_samples": [], "vote_score": 0.0}

def blank_cell(row, col):
    return {
        "row": row,
        "col": col,
        "color": "无",
        "confidence": 0.0,
        "samples": [],
    }

def merge_scan_sample(cell, sample):
    cell["samples"].append(sample)
    if sample["color"] != "无" and sample["confidence"] > cell["confidence"]:
        cell["color"] = sample["color"]
        cell["confidence"] = sample["confidence"]
        cell["box"] = sample.get("box")
        cell["pose"] = sample.get("pose")
        cell["center"] = sample.get("center")
        cell["anchor"] = sample.get("anchor")
    elif cell["color"] == "无" and sample["confidence"] > cell["confidence"]:
        cell["confidence"] = sample["confidence"]
        cell["box"] = sample.get("box")
        cell["pose"] = sample.get("pose")
        cell["center"] = sample.get("center")
        cell["anchor"] = sample.get("anchor")

def scan_board_state():
    center_pose = clamp_angles(FIXED_BOARD_CENTER_POSE)
    origin = read_actual_angles()
    targets = active_scan_targets(center_pose)
    fused = [[blank_cell(row + 1, col + 1) for col in range(3)] for row in range(3)]
    scans = []

    with arm_lock:
        try:
            for target in targets:
                _move_to_unlocked(target["pose"], ACTIVE_SCAN_MOVE_MS)
                time.sleep(ACTIVE_SCAN_SETTLE_SEC)
                frame = get_latest_frame(timeout=3.0)
                board = analyze_board_frame(frame)
                x1, y1, x2, y2 = central_sample_box(frame)
                frame_samples = []
                for frame_index in range(ACTIVE_SCAN_SAMPLE_FRAMES):
                    if frame_index > 0:
                        time.sleep(ACTIVE_SCAN_SAMPLE_INTERVAL_SEC)
                        frame = get_latest_frame(timeout=1.0)
                    frame_sample = classify_board_cell(frame, x1, y1, x2, y2)
                    frame_sample["frame_index"] = frame_index + 1
                    frame_samples.append(frame_sample)
                sample = choose_scan_sample(frame_samples)
                if sample["color"] != "无" and sample["confidence"] < FIXED_SCAN_MIN_CONFIDENCE:
                    sample["color"] = "无"
                time.sleep(ACTIVE_SCAN_AFTER_SAMPLE_SEC)
                sample.update({
                    "row": target["row"],
                    "col": target["col"],
                    "label": target.get("label"),
                    "box": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                    "pose": target["pose"],
                    "index": target["index"],
                    "offset": target["offset"],
                    "board_score": board.get("score"),
                    "board_complete": board.get("complete"),
                    "scan_mode": "fixed_pose_center_crop",
                })
                scans.append(sample)
                merge_scan_sample(fused[target["row"] - 1][target["col"] - 1], sample)
        finally:
            _move_to_unlocked(center_pose, ACTIVE_SCAN_MOVE_MS)

    merged_detections = [sample for sample in scans if sample.get("color") != "无"]

    cells = []
    for row in fused:
        result_row = []
        for cell in row:
            result = {
                "row": cell["row"],
                "col": cell["col"],
                "color": cell["color"],
                "confidence": round(cell["confidence"], 4),
                "samples": cell["samples"],
            }
            if "box" in cell:
                result["box"] = cell["box"]
            if "pose" in cell:
                result["pose"] = cell["pose"]
            if "center" in cell:
                result["center"] = cell["center"]
            if "anchor" in cell:
                result["anchor"] = cell["anchor"]
            result_row.append(result)
        cells.append(result_row)

    return {
        "cells": cells,
        "scan_count": len(scans),
        "scans": scans,
        "merged_detections": merged_detections,
        "min_confidence": FIXED_SCAN_MIN_CONFIDENCE,
        "center_pose": center_pose,
        "returned_to": center_pose,
        "origin": origin,
        "using_active_scan": True,
        "description": ["第%d行：%s" % (i + 1, "，".join(cell["color"] for cell in row)) for i, row in enumerate(cells)],
    }

def board_scan_candidates(origin):
    base = clamp_angles(origin)
    candidates = []
    seen = set()
    for s1 in [0, -10, 10, -18, 18, -26, 26]:
        for s2 in [0, -8, 8, -14, 14]:
            for s4 in [0, -8, 8]:
                pose = base[:]
                pose[0] += s1
                pose[1] += s2
                pose[3] += s4
                pose = clamp_angles(pose)
                key = tuple(pose)
                if key not in seen:
                    seen.add(key)
                    candidates.append(pose)
    return candidates

def find_board_view():
    best = None
    origin = read_actual_angles()
    candidates = board_scan_candidates(origin)
    with arm_lock:
        for pose in candidates:
            _move_to_unlocked(pose, BOARD_SCAN_MOVE_MS)
            time.sleep(BOARD_SCAN_SETTLE_SEC)
            frame = get_latest_frame(timeout=3.0)
            result = analyze_board_frame(frame)
            if best is None or result["score"] > best["board"]["score"]:
                best = {"angles": pose[:], "board": result}
            if result["complete"] and result["score"] >= BOARD_SCORE_OK:
                break
        if best is None:
            raise RuntimeError("board scan produced no candidates")
        if not best["board"]["complete"]:
            _move_to_unlocked(origin, BOARD_SCAN_MOVE_MS)
            raise RuntimeError("未找到完整棋盘视角，已回到扫描前角度；最佳候选=%r" % best)
        _move_to_unlocked(best["angles"], BOARD_SCAN_MOVE_MS)
    save_board_view_pose(best["angles"], best["board"])
    best["saved_to"] = CALIBRATION_PATH
    return best

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        print("%s - %s" % (self.address_string(), fmt % args))

    def send_json(self, code, payload):
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def read_json(self):
        length = int(self.headers.get("Content-Length", "0") or "0")
        raw = self.rfile.read(length) if length else b"{}"
        return json.loads(raw.decode("utf-8") or "{}")

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if path in ("/", "/index.html"):
            data = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return
        if path == "/video_feed":
            self.stream_video()
            return
        if path == "/api/camera_status":
            with camera_lock:
                age = None if latest_frame_time == 0 else round(time.time() - latest_frame_time, 3)
                ok = latest_frame is not None and age is not None and age < 2.0
                err = latest_frame_error
            self.send_json(200, {"status": "ok" if ok else "error", "frame_age": age, "message": err})
            return
        if path == "/api/board_view":
            try:
                frame = get_latest_frame(timeout=3.0)
                board = analyze_board_frame(frame)
                self.send_json(200, {"status": "ok", "board": board})
            except Exception as exc:
                self.send_json(500, {"status": "error", "message": repr(exc)})
            return
        if path == "/api/scan_board":
            try:
                result = scan_board_state()
                self.send_json(200, {
                    "status": "ok",
                    "cells": result["cells"],
                    "scan_count": result["scan_count"],
                    "scans": result["scans"],
                    "center_pose": result["center_pose"],
                    "returned_to": result["returned_to"],
                    "origin": result["origin"],
                    "using_active_scan": result["using_active_scan"],
                    "description": result["description"],
                })
            except Exception as exc:
                self.send_json(500, {"status": "error", "message": repr(exc)})
            return
        if path == "/api/angles":
            try:
                angles = read_actual_angles()
                self.send_json(200, {"status": "ok", "angles": angles, "limits": {"min": SERVO_MIN, "max": SERVO_MAX}})
            except Exception as exc:
                self.send_json(500, {"status": "error", "message": repr(exc)})
            return
        if path == "/api/pick_locations":
            self.send_json(200, {"status": "ok", **list_pick_locations()})
            return
        if path == "/api/cell_pose_memory":
            self.send_json(200, {
                "status": "ok",
                "path": BOARD_CELL_POSE_PATH,
                "cells": [pick_location_payload(key) for key in BOARD_CELL_KEYS if key in PICK_LOCATIONS],
            })
            return
        if path == "/api/track2_state":
            self.send_json(200, track2_status_response())
            return
        if path == "/api/detect_pick_angle":
            try:
                query = urllib.parse.parse_qs(parsed.query)
                cell_key = query.get("cell", ["cell_22"])[0]
                result = debug_detect_pick_angle(cell_key)
                self.send_json(200, result)
            except Exception as exc:
                self.send_json(500, {"status": "error", "message": repr(exc)})
            return
        if path == "/api/health":
            self.send_json(200, {"status": "ok", "angles": current_angles})
            return
        self.send_json(404, {"status": "error", "message": "not found"})

    def stream_video(self):
        try:
            cv2 = import_cv2()
            start_camera_thread()
        except Exception as exc:
            self.send_json(503, {"status": "error", "message": repr(exc)})
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        try:
            while True:
                frame = get_latest_frame(timeout=3.0)
                ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if not ok:
                    continue
                jpg = encoded.tobytes()
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(("Content-Length: %d\r\n\r\n" % len(jpg)).encode("ascii"))
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / CAMERA_FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception:
            pass

    def do_POST(self):
        try:
            payload = self.read_json()
            if self.path == "/api/angles":
                angles = move_to(payload.get("angles"), payload.get("time", MOVE_MS_DEFAULT))
                self.send_json(200, {"status": "ok", "angles": angles})
                return
            if self.path == "/api/home":
                angles = move_to(HOME, 800)
                self.send_json(200, {"status": "ok", "angles": angles})
                return
            if self.path == "/api/dance":
                angles = run_dance()
                self.send_json(200, {"status": "ok", "angles": angles})
                return
            if self.path == "/api/find_board_view":
                result = find_board_view()
                self.send_json(200, {"status": "ok", "angles": result["angles"], "board": result["board"], "saved_to": result["saved_to"]})
                return
            if self.path == "/api/manual_pick_place":
                result = run_manual_pick_place(payload.get("pick_pose"), payload.get("place_pose"), payload.get("pick_current") is True)
                self.send_json(200, result)
                return
            if self.path == "/api/pick_place":
                result = run_pick_place(payload.get("source"), payload.get("target"))
                self.send_json(200, result)
                return
            if self.path == "/api/track2_prepare_start":
                result = run_track2_prepare_start(payload)
                self.send_json(200, result)
                return
            if self.path == "/api/track2_start":
                result = run_track2_start(payload)
                self.send_json(200, result)
                return
            if self.path == "/api/track2_next":
                result = run_track2_next(payload)
                self.send_json(200, result)
                return
            if self.path == "/api/track2_reset":
                result = run_track2_reset(payload)
                self.send_json(200, result)
                return
            if self.path == "/api/center_grab_test":
                result = run_center_grab_test()
                self.send_json(200, result)
                return
            if self.path == "/api/center_place_test":
                result = run_center_place_test()
                self.send_json(200, result)
                return
            if self.path == "/api/center_pick_place":
                result = run_center_pick_place_cycle()
                self.send_json(200, result)
                return
            self.send_json(404, {"status": "error", "message": "not found"})
        except Exception as exc:
            self.send_json(400, {"status": "error", "message": repr(exc)})

class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True

if __name__ == "__main__":
    try:
        load_board_cell_poses()
    except Exception as exc:
        print("Failed to load board cell poses: %r" % exc, flush=True)
    print("Starting DOFBOT web controller on http://0.0.0.0:%d" % PORT, flush=True)
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    server.serve_forever()
