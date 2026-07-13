import threading
import time
from typing import Dict, Optional, Tuple

import cv2
import numpy as np


class VisionState:
    def __init__(self):
        self.face: Optional[Dict] = None
        self.gesture: Optional[str] = None
        self.message = "idle"
        self.mediapipe_available = False
        self.frame_ok = False

    def as_dict(self) -> Dict:
        return {
            "face": self.face,
            "gesture": self.gesture,
            "message": self.message,
            "mediapipe_available": self.mediapipe_available,
            "frame_ok": self.frame_ok,
        }


class CameraProcessor:
    def __init__(self, action_manager=None, camera_index: int = 0):
        self.action_manager = action_manager
        self.camera_index = camera_index
        self.capture = cv2.VideoCapture(camera_index)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc("M", "J", "P", "G"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        self.lock = threading.Lock()
        self.state = VisionState()
        self.last_gesture_trigger = 0.0
        self.face_cascade = None
        try:
            cascade_path = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
            if hasattr(cv2, "CascadeClassifier"):
                cascade = cv2.CascadeClassifier(cascade_path)
                if not cascade.empty():
                    self.face_cascade = cascade
        except Exception:
            self.face_cascade = None
        self.mp_hands = None
        self.mp_draw = None
        self.hands = None
        try:
            import mediapipe as mp  # type: ignore

            self.mp_hands = mp.solutions.hands
            self.mp_draw = mp.solutions.drawing_utils
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.6,
                min_tracking_confidence=0.6,
            )
            self.state.mediapipe_available = True
        except Exception:
            self.state.mediapipe_available = False

    def release(self):
        if self.capture:
            self.capture.release()

    def get_processed_frame(self) -> Tuple[bool, np.ndarray]:
        with self.lock:
            ok, frame = self.capture.read()
        if not ok or frame is None:
            self.state.frame_ok = False
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "Camera unavailable", (160, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (80, 80, 255), 2)
            return False, frame

        frame = cv2.resize(frame, (640, 480))
        self.state.frame_ok = True
        self._process_face(frame)
        self._process_gesture(frame)
        self._draw_overlay(frame)
        return True, frame

    def jpeg_stream(self):
        while True:
            _, frame = self.get_processed_frame()
            ok, buffer = cv2.imencode(".jpg", frame)
            if ok:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
                )
            time.sleep(0.03)

    def _process_face(self, frame):
        if not self.action_manager or not self.action_manager.face_tracking:
            self.state.face = None
            return
        if self.face_cascade is None:
            self.state.face = None
            self.state.message = "face detector unavailable"
            cv2.putText(frame, "Face detector unavailable", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (245, 158, 11), 2)
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=1.3, minNeighbors=5)
        if len(faces) == 0:
            self.state.face = None
            return
        x, y, w, h = max(faces, key=lambda item: item[2] * item[3])
        cx = int(x + w / 2)
        cy = int(y + h / 2)
        offset_x = cx - 320
        offset_y = cy - 240
        self.state.face = {
            "x": int(x),
            "y": int(y),
            "w": int(w),
            "h": int(h),
            "offset_x": int(offset_x),
            "offset_y": int(offset_y),
        }
        cv2.rectangle(frame, (x, y), (x + w, y + h), (34, 197, 94), 2)
        cv2.line(frame, (320, 240), (cx, cy), (34, 197, 94), 2)
        cv2.circle(frame, (cx, cy), 4, (34, 197, 94), -1)

    def _process_gesture(self, frame):
        if not self.action_manager or not self.action_manager.gesture_enabled:
            self.state.gesture = None
            return
        if not self.hands:
            self.state.message = "mediapipe unavailable"
            return
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)
        if not result.multi_hand_landmarks:
            self.state.gesture = None
            return
        hand = result.multi_hand_landmarks[0]
        if self.mp_draw:
            self.mp_draw.draw_landmarks(frame, hand, self.mp_hands.HAND_CONNECTIONS)
        points = []
        h, w, _ = frame.shape
        for landmark in hand.landmark:
            points.append((int(landmark.x * w), int(landmark.y * h)))
        gesture = classify_hand(points)
        self.state.gesture = gesture
        cv2.putText(frame, gesture or "hand", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (59, 130, 246), 2)
        if gesture:
            action = gesture_to_action(gesture)
            now = time.time()
            if action and now - self.last_gesture_trigger > 2.0:
                self.last_gesture_trigger = now
                threading.Thread(
                    target=self.action_manager.run_action,
                    args=(action, "gesture"),
                    daemon=True,
                ).start()

    def _draw_overlay(self, frame):
        cv2.line(frame, (300, 240), (340, 240), (148, 163, 184), 1)
        cv2.line(frame, (320, 220), (320, 260), (148, 163, 184), 1)
        status = "mode: idle"
        if self.action_manager:
            status = f"mode: {self.action_manager.current_mode}"
        cv2.putText(frame, status, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (248, 250, 252), 2)

    def status(self) -> Dict:
        return self.state.as_dict()


def classify_hand(points):
    if len(points) < 21:
        return None
    fingers = fingers_up(points)
    total = sum(fingers)
    if total == 5:
        return "open_palm"
    if total == 1 and fingers[1] == 1:
        return "one"
    if total == 2 and fingers[1] == 1 and fingers[2] == 1:
        return "victory"
    if distance(points[4], points[8]) < 35 and fingers[2] == fingers[3] == fingers[4] == 1:
        return "ok"
    if points[4][1] > points[3][1] and total <= 1:
        return "thumb_down"
    return None


def fingers_up(points):
    fingers = []
    fingers.append(1 if points[4][0] > points[3][0] else 0)
    for tip in (8, 12, 16, 20):
        fingers.append(1 if points[tip][1] < points[tip - 2][1] else 0)
    return fingers


def distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def gesture_to_action(gesture):
    return {
        "open_palm": "applaud",
        "ok": "wave",
        "one": "nod",
        "victory": "dance",
        "thumb_down": "stop",
    }.get(gesture)
