import threading
import time
from typing import Dict, List, Optional


DEFAULT_HOME = [90, 135, 0, 45, 90, 90]


class RobotArm:
    def __init__(self):
        self.lock = threading.Lock()
        self.simulation = False
        self.last_joints = DEFAULT_HOME[:]
        self.log = []
        try:
            from Arm_Lib import Arm_Device  # type: ignore

            self.arm = Arm_Device()
            time.sleep(0.1)
        except Exception as exc:
            self.arm = None
            self.simulation = True
            self.log_event("simulation", f"Arm_Lib unavailable: {exc}")

    def log_event(self, event: str, message: str):
        self.log.insert(
            0,
            {
                "time": time.strftime("%H:%M:%S"),
                "event": event,
                "message": message,
            },
        )
        self.log = self.log[:80]

    def status(self) -> Dict:
        return {
            "simulation": self.simulation,
            "last_joints": self.last_joints,
            "log": self.log[:20],
        }

    def move_to_joints(self, joints: List[float], duration: int = 800) -> Dict:
        if len(joints) != 6:
            return {"ok": False, "msg": "joints must contain 6 values"}
        clamped = [
            max(0, min(270 if index == 4 else 180, float(value)))
            for index, value in enumerate(joints)
        ]
        with self.lock:
            self.last_joints = clamped
            if self.arm is not None:
                self.arm.Arm_serial_servo_write6(
                    clamped[0],
                    clamped[1],
                    clamped[2],
                    clamped[3],
                    clamped[4],
                    clamped[5],
                    int(duration),
                )
            else:
                time.sleep(min(max(duration / 1000.0, 0.05), 0.4))
            self.log_event("move", f"joints={clamped}, duration={duration}ms")
        return {"ok": True, "joints": clamped, "duration": duration}

    def read_current_joints(self) -> List[float]:
        if self.arm is None:
            return self.last_joints[:]
        joints = []
        for servo_id in range(1, 7):
            angle = self.arm.Arm_serial_servo_read(servo_id)
            joints.append(self.last_joints[servo_id - 1] if angle is None else angle)
            time.sleep(0.005)
        self.last_joints = joints
        return joints

    def single_servo(self, servo_id: int, angle: float, duration: int = 500) -> Dict:
        if not 1 <= servo_id <= 6:
            return {"ok": False, "msg": "servo_id must be 1..6"}
        limit = 270 if servo_id == 5 else 180
        angle = max(0, min(limit, float(angle)))
        with self.lock:
            self.last_joints[servo_id - 1] = angle
            if self.arm is not None:
                self.arm.Arm_serial_servo_write(servo_id, angle, int(duration))
            else:
                time.sleep(min(max(duration / 1000.0, 0.05), 0.3))
            self.log_event("servo", f"S{servo_id}={angle}, duration={duration}ms")
        return {"ok": True, "servo_id": servo_id, "angle": angle}

    def buzzer(self, delay: int = 1):
        if self.arm is not None:
            self.arm.Arm_Buzzer_On(delay)
        self.log_event("buzzer", f"delay={delay}")

    def stop(self):
        self.log_event("stop", "stop requested")


class ActionManager:
    def __init__(self, arm: RobotArm):
        self.arm = arm
        self.running = False
        self.current_mode = "idle"
        self.current_action: Optional[str] = None
        self.lock = threading.Lock()
        self.face_tracking = False
        self.gesture_enabled = False
        self.last_source = "system"

    def status(self) -> Dict:
        arm_status = self.arm.status()
        return {
            "running": self.running,
            "current_mode": self.current_mode,
            "current_action": self.current_action,
            "face_tracking": self.face_tracking,
            "gesture_enabled": self.gesture_enabled,
            "last_source": self.last_source,
            **arm_status,
        }

    def run_action(self, action_name: str, source: str = "web") -> Dict:
        with self.lock:
            if self.running:
                return {"ok": False, "msg": "机械臂正在执行动作", "action": self.current_action}
            self.running = True
            self.current_action = action_name
            self.last_source = source
        try:
            result = self._execute(action_name)
            return {"ok": True, "action": action_name, "source": source, **result}
        except Exception as exc:
            self.arm.log_event("error", str(exc))
            return {"ok": False, "msg": str(exc), "action": action_name}
        finally:
            with self.lock:
                self.running = False
                if action_name not in ("face_follow_on", "gesture_on"):
                    self.current_action = None

    def stop(self) -> Dict:
        self.face_tracking = False
        self.gesture_enabled = False
        self.current_mode = "idle"
        self.current_action = None
        self.arm.stop()
        return {"ok": True, "mode": self.current_mode}

    def _execute(self, action_name: str) -> Dict:
        if action_name == "home":
            return self.arm.move_to_joints(DEFAULT_HOME, 1000)
        if action_name == "wave":
            return self._sequence(
                [
                    [70, 130, 20, 25, 90, 90],
                    [115, 130, 20, 25, 90, 90],
                    [70, 130, 20, 25, 90, 90],
                    DEFAULT_HOME,
                ],
                450,
            )
        if action_name == "nod":
            return self._sequence(
                [
                    [90, 120, 20, 20, 90, 90],
                    [90, 145, 0, 45, 90, 90],
                    [90, 120, 20, 20, 90, 90],
                    DEFAULT_HOME,
                ],
                450,
            )
        if action_name == "applaud":
            return self._sequence(
                [
                    [90, 135, 0, 70, 90, 30],
                    [90, 135, 0, 70, 90, 180],
                    [90, 135, 0, 70, 90, 30],
                    DEFAULT_HOME,
                ],
                400,
            )
        if action_name == "dance":
            return self._sequence(
                [
                    [90, 90, 90, 90, 90, 90],
                    [90, 60, 120, 60, 90, 90],
                    [90, 120, 60, 60, 90, 90],
                    [45, 90, 90, 90, 90, 120],
                    [135, 90, 90, 90, 90, 120],
                    DEFAULT_HOME,
                ],
                500,
            )
        if action_name == "gesture_on":
            self.gesture_enabled = True
            self.face_tracking = False
            self.current_mode = "gesture"
            self.arm.log_event("mode", "gesture recognition enabled")
            return {"mode": self.current_mode}
        if action_name == "face_follow_on":
            self.face_tracking = True
            self.gesture_enabled = False
            self.current_mode = "face_follow"
            self.arm.log_event("mode", "face tracking enabled")
            return {"mode": self.current_mode}
        if action_name == "stop":
            return self.stop()
        raise ValueError(f"unknown action: {action_name}")

    def _sequence(self, poses: List[List[float]], duration: int) -> Dict:
        for pose in poses:
            self.arm.move_to_joints(pose, duration)
            time.sleep(duration / 1000.0 + 0.05)
        return {"steps": len(poses)}


INTENTS = {
    "回家": "home",
    "复位": "home",
    "归位": "home",
    "挥手": "wave",
    "打招呼": "wave",
    "你好": "wave",
    "点头": "nod",
    "同意": "nod",
    "鼓掌": "applaud",
    "拍手": "applaud",
    "跳舞": "dance",
    "表演": "dance",
    "看着我": "face_follow_on",
    "人脸": "face_follow_on",
    "追踪": "face_follow_on",
    "手势": "gesture_on",
    "手势控制": "gesture_on",
    "停止": "stop",
    "停下": "stop",
}


def parse_intent(text: str) -> Optional[str]:
    normalized = text.strip().lower()
    for keyword, action in INTENTS.items():
        if keyword.lower() in normalized:
            return action
    return None

