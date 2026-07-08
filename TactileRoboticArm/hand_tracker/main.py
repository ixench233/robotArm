# main.py
import cv2
import numpy as np
import time
import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor

from robot_control.config import Config
from robot_control.tracker import DepthCameraTracker
from robot_control.kinematics import RobotKinematics
from robot_control.communicator import RobotCommunicator
import robot_control.utils as utils

def on_send_done(future):
    try:
        was_scheduled = future.result()
        if not was_scheduled:
            print("[Debug] Send task was not scheduled (connection likely closed).")
    except Exception as e:
        print(f"[Debug] Exception occurred in scheduled send task: {e}")

class BlockingWorker:
    def __init__(self, cfg, communicator, loop, shutdown_event, plot_queue, robot_kinematics):
        self.cfg = cfg
        self.communicator = communicator
        self.loop = loop
        self.shutdown_event = shutdown_event
        self.plot_queue = plot_queue
        self.robot_kinematics = robot_kinematics
        self.tracker = None

    def setup(self):
        try:
            self.tracker = DepthCameraTracker()
            return True
        except Exception as e:
            print(f"FATAL: Could not initialize DepthCameraTracker. {e}")
            return False

    def run(self):
        if not self.setup():
            self.shutdown_event.set()
            return
        SEND_INTERVAL = self.cfg.TRANSMISSION_INTERVAL
        home_pos_rad = [np.deg2rad(a) for a in self.cfg.HOME_POSITION_DEG]
        current_angles = [0.0] * len(self.robot_kinematics.my_chain.links)
        current_angles[1:7] = home_pos_rad
        initial_fk = self.robot_kinematics.my_chain.forward_kinematics(current_angles)
        smoothed_target = initial_fk[:3, 3]
        smoothed_j5_rad = current_angles[5]
        last_send_time = 0
        last_sent_angles_deg = []
        final_angles_deg = self.cfg.HOME_POSITION_DEG
        ik_msg = "Initializing"
        ik_ok, paused = True, False
        try:
            while not self.shutdown_event.is_set():
                hand_results, depth_frame, depth_intrin, frame = self.tracker.get_frames_and_process_hands()
                if frame is None:
                    time.sleep(0.1)
                    continue
                j6_rad = current_angles[6]
                j5_rad = smoothed_j5_rad
                if hand_results and hand_results.multi_hand_landmarks:
                    lm = hand_results.multi_hand_landmarks[0]
                    self.tracker.draw_landmarks(frame, lm)
                    wrist_landmark = lm.landmark[self.tracker.mp_hands.HandLandmark.WRIST]
                    point_3d_cam = utils.get_3d_camera_coordinate(wrist_landmark, depth_frame, depth_intrin)
                    if point_3d_cam is not None:
                        raw_target = utils.map_camera_to_workspace_by_interpolation(point_3d_cam, self.cfg)
                        if raw_target is not None:
                            smoothed_target = (1 - self.cfg.SMOOTHING_FACTOR) * smoothed_target + self.cfg.SMOOTHING_FACTOR * raw_target
                        h, w, _ = frame.shape
                        wrist_px = int(wrist_landmark.x * w)
                        wrist_py = int(wrist_landmark.y * h)
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cam_text = f"Cam: [{point_3d_cam[0]:.2f}, {point_3d_cam[1]:.2f}, {point_3d_cam[2]:.2f}]"
                        cv2.putText(frame, cam_text, (wrist_px + 10, wrist_py), font, 0.5, (255, 255, 0), 2)
                        if raw_target is not None:
                            robot_text = f"Robot: [{raw_target[0]:.2f}, {raw_target[1]:.2f}, {raw_target[2]:.2f}]"
                            cv2.putText(frame, robot_text, (wrist_px + 10, wrist_py + 20), font, 0.5, (0, 255, 255), 2)
                    raw_hand_rot_deg = utils.get_hand_rotation_angle_deg(lm)
                    target_j5_deg = np.interp(raw_hand_rot_deg, [self.cfg.HAND_ROTATION_MIN_DEG, self.cfg.HAND_ROTATION_MAX_DEG], [self.cfg.J5_TARGET_MIN_DEG, self.cfg.J5_TARGET_MAX_DEG])
                    target_j5_deg = np.clip(target_j5_deg, self.cfg.J5_TARGET_MIN_DEG, self.cfg.J5_TARGET_MAX_DEG)
                    j5_rad = np.deg2rad(target_j5_deg)
                    smoothed_j5_rad = (1 - self.cfg.J5_SMOOTHING_FACTOR) * smoothed_j5_rad + self.cfg.J5_SMOOTHING_FACTOR * j5_rad
                    hand_open_dist = utils.get_hand_openness_distance(lm)
                    j6_deg = np.interp(hand_open_dist, [self.cfg.HAND_OPEN_MIN_DIST, self.cfg.HAND_OPEN_MAX_DIST], [self.cfg.J6_ANGLE_MAX_DEG, self.cfg.J6_ANGLE_MIN_DEG])
                    j6_rad = np.deg2rad(np.clip(j6_deg, self.cfg.J6_ANGLE_MIN_DEG, self.cfg.J6_ANGLE_MAX_DEG))
                target_pos = smoothed_target
                ik_angles, ik_ok = self.robot_kinematics.solve_ik(target_pos, current_angles)
                ik_msg = "OK" if ik_ok else "Failed"
                if ik_ok: current_angles = ik_angles
                current_angles[5] = smoothed_j5_rad
                current_angles[6] = j6_rad
                calculated_angles_deg = [np.degrees(a) for a in current_angles[1:7]]
                mapped_angles = utils.apply_servo_mapping(calculated_angles_deg, self.cfg.SERVO_K, self.cfg.SERVO_B)
                final_angles_deg = [int(round(a)) for a in mapped_angles]
                current_time = time.time()
                if not self.cfg.SIMULATION_ONLY and not paused:
                    time_to_send = (current_time - last_send_time) > SEND_INTERVAL
                    data_has_changed = (final_angles_deg != last_sent_angles_deg)
                    if time_to_send and data_has_changed:
                        future = asyncio.run_coroutine_threadsafe(
                            self.communicator.send_data_binary(final_angles_deg), 
                            self.loop
                        )
                        future.add_done_callback(on_send_done)
                        last_sent_angles_deg = final_angles_deg
                        last_send_time = current_time
                if not paused:
                    try:
                        self.plot_queue.put_nowait((current_angles, smoothed_target))
                    except asyncio.QueueFull:
                        pass
                frame = utils.draw_info_on_frame(frame, final_angles_deg, ik_msg, ik_ok, smoothed_target, paused)
                cv2.imshow('Robot Arm 3D Control Panel', frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): self.shutdown_event.set()
                elif key == ord('p'): paused = not paused; print(f"Simulation {'PAUSED' if paused else 'RESUMED'}.")
                elif key == ord('r'):
                    current_angles[1:7] = home_pos_rad
                    smoothed_target = self.robot_kinematics.my_chain.forward_kinematics(current_angles)[:3, 3]
                    smoothed_j5_rad = current_angles[5]
                    last_sent_angles_deg.clear()
                    print("--- Position Reset ---")
        finally:
            print("Worker thread cleaning up resources...")
            cv2.destroyAllWindows()
            if self.tracker:
                self.tracker.close()
            
async def plot_updater(plot_queue, robot_kinematics, shutdown_event):
    print("Plot updater started.")
    while not shutdown_event.is_set():
        try:
            angles, target = await asyncio.wait_for(plot_queue.get(), timeout=0.1)
            robot_kinematics.update_plot(angles, target)
            plot_queue.task_done()
        except asyncio.TimeoutError:
            continue
        except asyncio.CancelledError:
            break
    print("Plot updater closed.")

async def main():
    cfg = Config()
    loop = asyncio.get_running_loop()
    shutdown_event = threading.Event()
    plot_queue = asyncio.Queue(maxsize=1)
    workspace = {'x': (cfg.ROBOT_X_MIN, cfg.ROBOT_X_MAX),
                 'y': (cfg.ROBOT_Y_MIN, cfg.ROBOT_Y_MAX),
                 'z': (cfg.ROBOT_Z_MIN, cfg.ROBOT_Z_MAX)}
    robot_kinematics = RobotKinematics(urdf_file_path="urdf/dofbot.urdf", workspace_limits=workspace)
    communicator = None
    if not cfg.SIMULATION_ONLY:
        communicator = RobotCommunicator(cfg.SERVER_URI)
        await communicator.connect()
    else:
        print("\n" + "="*40 + "\n   Running in SIMULATION-ONLY mode.\n" + "="*40)
    worker = BlockingWorker(cfg, communicator, loop, shutdown_event, plot_queue, robot_kinematics)
    executor = ThreadPoolExecutor(max_workers=1)
    plot_task = loop.create_task(plot_updater(plot_queue, robot_kinematics, shutdown_event))
    try:
        print("Starting worker thread...")
        blocking_task = loop.run_in_executor(executor, worker.run)
        await blocking_task
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\nMain program received interrupt signal.")
    finally:
        print("Main program shutting down...")
        shutdown_event.set()
        
        plot_task.cancel()
        await asyncio.gather(plot_task, return_exceptions=True)

        executor.shutdown(wait=True)
        if communicator:
            await communicator.close()
            
        robot_kinematics.close()
        print("All resources cleaned up. Program exiting.")

if __name__ == "__main__":
    try:
        import pyrealsense2
    except ImportError:
        print("\nERROR: pyrealsense2 is not installed.")
        print("Please install it using: pip install pyrealsense2\n")
    else:
        try:
            asyncio.run(main())
        except KeyboardInterrupt:
            print("\nProgram interrupted by user.")