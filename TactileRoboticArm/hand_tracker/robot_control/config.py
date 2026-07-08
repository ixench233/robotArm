# robot_control/config.py
class Config:
    """Holds all configuration parameters for the application."""
    # --- Operation Mode ---
    SIMULATION_ONLY = False
    SERVER_URI = "ws://192.168.21.200:8765"


    # --- Robot Workspace Limits (Destination Box) ---
    ROBOT_X_MIN, ROBOT_X_MAX = -0.25, 0.25
    ROBOT_Y_MIN, ROBOT_Y_MAX = 0.05, 0.32
    ROBOT_Z_MIN, ROBOT_Z_MAX = 0.03, 0.25

    # --- 3D Camera-to-Workspace Mapping (Source Box) ---
    CAM_SPACE_X_MIN, CAM_SPACE_X_MAX = -0.20, 0.20  # 相机左右 (Camera X)
    CAM_SPACE_Y_MIN, CAM_SPACE_Y_MAX = -0.15, 0.25  # 相机上下 (Camera Y)
    CAM_SPACE_Z_MIN, CAM_SPACE_Z_MAX = 0.30, 0.60  # 相机远近 (Camera Z)

    # --- 坐标映射与方向控制 ---
    
    # 1. 坐标轴映射 (Transpose)
    # 定义: [机器人X来自哪个相机轴, 机器人Y来自哪个相机轴, 机器人Z来自哪个相机轴]
    # 相机轴: 0=X, 1=Y, 2=Z
    COORDINATE_AXIS_MAP = [0, 1, 2]

    # 2. 坐标极性/方向控制 (Polarity)
    # 定义: [机器人X轴极性, 机器人Y轴极性, 机器人Z轴极性]
    #   1: 正向 (相机控制区坐标增大 -> 机器人工作区坐标增大)
    #  -1: 反向 (相机控制区坐标增大 -> 机器人工作区坐标减小)
    COORDINATE_POLARITY = [-1, -1, 1]
    
    # --- Control Tuning ---
    SMOOTHING_FACTOR = 0.3
    TRANSMISSION_INTERVAL = 0.05

    # --- Joint 6 Control (Gripper) ---
    J6_ANGLE_MIN_DEG, J6_ANGLE_MAX_DEG = 40, 120
    HAND_OPEN_MIN_DIST, HAND_OPEN_MAX_DIST = 0.05, 0.20

    # --- Joint 5 Control (Hand Rotation) ---
    J5_SMOOTHING_FACTOR = 0.5
    HAND_ROTATION_MIN_DEG = -45.0
    HAND_ROTATION_MAX_DEG = 45.0
    J5_TARGET_MIN_DEG = 0.0
    J5_TARGET_MAX_DEG = 180.0

    # --- Initial State ---
    HOME_POSITION_DEG = [0, 0, 0, 0, 0, 40]

    # --- Servo Angle Mapping (y = kx + b) ---
    SERVO_K = [1, 1, 1, 1, -1, 1]
    SERVO_B = [90, 90, 90, 90, 180, 0.0]