# robot_control/utils.py
import cv2
import numpy as np
import math
import mediapipe as mp
import pyrealsense2 as rs

def get_3d_camera_coordinate(lm_point, depth_frame, depth_intrin):
    pixel_x = int(lm_point.x * depth_frame.get_width())
    pixel_y = int(lm_point.y * depth_frame.get_height())
    if not (0 <= pixel_x < depth_frame.get_width() and 0 <= pixel_y < depth_frame.get_height()):
        return None
    depth_m = depth_frame.get_distance(pixel_x, pixel_y)
    if depth_m <= 0:
        return None

    point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [pixel_x, pixel_y], depth_m)
    return np.array(point_3d)

def map_camera_to_workspace_by_interpolation(point_3d_camera, cfg):
    if point_3d_camera is None:
        return None
    cam_ranges = [
        [cfg.CAM_SPACE_X_MIN, cfg.CAM_SPACE_X_MAX],
        [cfg.CAM_SPACE_Y_MIN, cfg.CAM_SPACE_Y_MAX],
        [cfg.CAM_SPACE_Z_MIN, cfg.CAM_SPACE_Z_MAX]
    ]
    robot_base_mins = [cfg.ROBOT_X_MIN, cfg.ROBOT_Y_MIN, cfg.ROBOT_Z_MIN]
    robot_base_maxs = [cfg.ROBOT_X_MAX, cfg.ROBOT_Y_MAX, cfg.ROBOT_Z_MAX]
    robot_point = np.zeros(3)
    for robot_axis_idx in range(3):
        cam_axis_idx = cfg.COORDINATE_AXIS_MAP[robot_axis_idx]
        cam_value = point_3d_camera[cam_axis_idx]
        cam_range = cam_ranges[cam_axis_idx]
        polarity = cfg.COORDINATE_POLARITY[robot_axis_idx]
        if polarity == 1:
            robot_range = [robot_base_mins[robot_axis_idx], robot_base_maxs[robot_axis_idx]]
        else:
            robot_range = [robot_base_maxs[robot_axis_idx], robot_base_mins[robot_axis_idx]]
        robot_point[robot_axis_idx] = np.interp(cam_value, cam_range, robot_range)
    clamped_robot_point = np.clip(
        robot_point,
        [cfg.ROBOT_X_MIN, cfg.ROBOT_Y_MIN, cfg.ROBOT_Z_MIN],
        [cfg.ROBOT_X_MAX, cfg.ROBOT_Y_MAX, cfg.ROBOT_Z_MAX]
    )
    return clamped_robot_point

def get_hand_openness_distance(lm):
    p1 = lm.landmark[mp.solutions.hands.HandLandmark.THUMB_TIP]
    p2 = lm.landmark[mp.solutions.hands.HandLandmark.PINKY_TIP]
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def get_hand_rotation_angle_deg(lm):
    p0 = lm.landmark[mp.solutions.hands.HandLandmark.WRIST]
    p9 = lm.landmark[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_MCP]
    vec_x = p9.x - p0.x
    vec_y = p9.y - p0.y
    angle_rad = math.atan2(vec_x, -vec_y)
    return math.degrees(angle_rad)

def apply_servo_mapping(angles_deg, k_vals, b_vals):
    if len(angles_deg) != len(k_vals) or len(angles_deg) != len(b_vals):
        print("Warning: Angle/K/B list lengths differ. Mapping skipped.")
        return angles_deg
    return [(k * x + b) for k, x, b in zip(k_vals, angles_deg, b_vals)]

def draw_info_on_frame(frame, angles, ik_msg, ik_ok, target, paused):
    y, font = 30, cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, f"IK Status: {ik_msg}", (10,y), font, 0.7, (0,255,0) if ik_ok else (0,0,255), 2)
    y+=40; cv2.putText(frame, f"Target XYZ: [{target[0]:.2f},{target[1]:.2f},{target[2]:.2f}] m",(10,y),font,0.7,(255,255,0),2)
    y+=40; cv2.putText(frame, "Final Angles (J1-J6, Deg):",(10,y),font,0.7,(0,255,0),2)
    y+=10
    for i, angle in enumerate(angles):
        cv2.putText(frame,f"Joint {i+1}: {angle}",(15,y+(i+1)*30),font,0.6,(0,255,0),2)
    cv2.putText(frame,"P:Pause | R:Reset | Q:Quit",(10,frame.shape[0]-10),font,0.6,(255,255,0),2)
    if paused:
        pause_text = "[PAUSED]"
        sz = cv2.getTextSize(pause_text, font, 1.5, 3)[0]
        cv2.putText(frame,pause_text,(frame.shape[1]-sz[0]-15,frame.shape[0]-15),font,1.5,(0,165,255),3)
    return frame