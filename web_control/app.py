import argparse
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, render_template, request

from action_store import ActionStore
from robot_arm import ActionManager, RobotArm, parse_intent
from vision import CameraProcessor


BASE_DIR = Path(__file__).resolve().parent

app = Flask(__name__, template_folder=str(BASE_DIR / "templates"), static_folder=str(BASE_DIR / "static"))
arm = RobotArm()
action_manager = ActionManager(arm)
camera = CameraProcessor(action_manager=action_manager)
store = ActionStore(str(BASE_DIR / "actions"))


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(camera.jpeg_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/status")
def status():
    return jsonify({"robot": action_manager.status(), "vision": camera.status(), "sequences": store.list_sequences()})


@app.route("/api/action", methods=["POST"])
def action():
    payload = request.get_json(force=True, silent=True) or {}
    action_name = payload.get("action")
    if not action_name:
        return jsonify({"ok": False, "msg": "missing action"}), 400
    result = action_manager.run_action(action_name, source=payload.get("source", "web"))
    return jsonify(result), 200 if result.get("ok") else 409


@app.route("/api/chat", methods=["POST"])
def chat():
    payload = request.get_json(force=True, silent=True) or {}
    text = payload.get("text", "")
    action_name = parse_intent(text)
    if not action_name:
        return jsonify({"ok": False, "reply": "未识别指令，可以试试：挥手、点头、鼓掌、跳舞、看着我、手势控制、停止。"})
    result = action_manager.run_action(action_name, source="chat")
    return jsonify({"ok": result.get("ok", False), "reply": f"识别到动作：{action_name}", "result": result})


@app.route("/api/mode", methods=["POST"])
def mode():
    payload = request.get_json(force=True, silent=True) or {}
    mode_name = payload.get("mode")
    mapping = {
        "idle": "stop",
        "face_follow": "face_follow_on",
        "gesture": "gesture_on",
    }
    action_name = mapping.get(mode_name)
    if not action_name:
        return jsonify({"ok": False, "msg": "unknown mode"}), 400
    result = action_manager.run_action(action_name, source="mode")
    return jsonify(result)


@app.route("/api/joints/current")
def current_joints():
    return jsonify({"ok": True, "joints": arm.read_current_joints()})


@app.route("/api/joints/move", methods=["POST"])
def move_joints():
    payload = request.get_json(force=True, silent=True) or {}
    result = arm.move_to_joints(payload.get("joints", []), int(payload.get("duration", 800)))
    return jsonify(result), 200 if result.get("ok") else 400


@app.route("/api/sequences", methods=["GET"])
def list_sequences():
    return jsonify({"ok": True, "sequences": store.list_sequences()})


@app.route("/api/sequences/<name>", methods=["GET"])
def get_sequence(name):
    try:
        return jsonify({"ok": True, "name": name, "steps": store.load(name)})
    except FileNotFoundError:
        return jsonify({"ok": False, "msg": "sequence not found"}), 404


@app.route("/api/sequences/<name>", methods=["POST"])
def save_sequence(name):
    payload = request.get_json(force=True, silent=True) or {}
    steps = payload.get("steps", [])
    if not isinstance(steps, list):
        return jsonify({"ok": False, "msg": "steps must be a list"}), 400
    store.save(name, steps)
    return jsonify({"ok": True, "name": name, "steps": steps})


@app.route("/api/sequences/<name>/record", methods=["POST"])
def record_step(name):
    payload = request.get_json(force=True, silent=True) or {}
    duration = int(payload.get("duration", 800))
    try:
        steps = store.load(name)
    except FileNotFoundError:
        steps = []
    joints = arm.read_current_joints()
    steps.append({"name": f"step_{len(steps) + 1}", "joints": joints, "duration": duration})
    store.save(name, steps)
    return jsonify({"ok": True, "name": name, "step": steps[-1], "steps": steps})


@app.route("/api/sequences/<name>/play", methods=["POST"])
def play_sequence(name):
    try:
        steps = store.load(name)
    except FileNotFoundError:
        return jsonify({"ok": False, "msg": "sequence not found"}), 404

    def runner():
        if action_manager.running:
            return
        action_manager.running = True
        action_manager.current_action = f"sequence:{name}"
        action_manager.last_source = "sequence"
        try:
            for step in steps:
                arm.move_to_joints(step.get("joints", []), int(step.get("duration", 800)))
                time.sleep(int(step.get("duration", 800)) / 1000.0 + 0.05)
        finally:
            action_manager.running = False
            action_manager.current_action = None

    threading.Thread(target=runner, daemon=True).start()
    return jsonify({"ok": True, "name": name, "steps": len(steps)})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=5000, type=int)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)


if __name__ == "__main__":
    main()

