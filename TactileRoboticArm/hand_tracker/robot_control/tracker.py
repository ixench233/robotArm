# robot_control/tracker.py
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np

class DepthCameraTracker:
    def __init__(self, min_detection_confidence=0.7, min_tracking_confidence=0.7):
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        print("Starting RealSense camera pipeline...")
        try:
            self.profile = self.pipeline.start(config)
        except Exception as e:
            print(f"Error starting RealSense pipeline: {e}")
            print("Please ensure the RealSense camera is connected and drivers are installed.")
            raise
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        self.depth_intrin = self.profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        print("Depth camera and hand tracker initialized successfully.")

    def get_frames_and_process_hands(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None, None, None
        color_image = np.asanyarray(color_frame.get_data())
        image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        image_rgb.flags.writeable = False
        hand_results = self.hands.process(image_rgb)
        image_rgb.flags.writeable = True
        return hand_results, depth_frame, self.depth_intrin, color_image

    def draw_landmarks(self, frame, hand_landmarks):
        self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

    def close(self):
        print("Closing camera and MediaPipe resources.")
        self.pipeline.stop()
        self.hands.close()