import numpy as np
import xml.etree.ElementTree as ET
import math
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

URDF_FILENAME = 'urdf/dofbot.urdf'

def rpy_to_rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def create_transformation_matrix(rpy, xyz):
    rotation_matrix = rpy_to_rotation_matrix(rpy)
    T = np.eye(4)
    T[:3, :3] = rotation_matrix
    T[:3, 3] = np.array(xyz)
    return T

def parse_urdf_for_kinematics(urdf_content):
    robot = ET.fromstring(urdf_content)
    joints = {}
    for joint in robot.findall('joint'):
        name = joint.get('name')
        joint_type = joint.get('type')
        limit_tag = joint.find('limit')
        if joint_type == 'revolute' and limit_tag is not None:
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            origin = joint.find('origin')
            xyz = [float(x) for x in origin.get('xyz').split()]
            rpy = [float(x) for x in origin.get('rpy').split()]
            axis_element = joint.find('axis')
            axis = [float(x) for x in axis_element.get('xyz').split()]
            
            joints[name] = {
                'parent': parent,
                'child': child,
                'xyz': xyz,
                'rpy': rpy,
                'axis': axis,
                'lower': float(limit_tag.get('lower')),
                'upper': float(limit_tag.get('upper'))
            }
    return joints

def forward_kinematics(joint_angles, joint_info, kinematic_chain):
    T = np.eye(4)
    for joint_name in kinematic_chain:
        if joint_name not in joint_info:
            continue
        info = joint_info[joint_name]
        angle = joint_angles.get(joint_name, 0)
        T_static = create_transformation_matrix(info['rpy'], info['xyz'])
        axis = np.array(info['axis'])
        c, s = math.cos(angle), math.sin(angle)
        ux, uy, uz = axis
        rotation_matrix = np.eye(4)
        rotation_matrix[:3, :3] = np.array([
            [c + ux*ux*(1-c), ux*uy*(1-c) - uz*s, ux*uz*(1-c) + uy*s],
            [uy*ux*(1-c) + uz*s, c + uy*uy*(1-c), uy*uz*(1-c) - ux*s],
            [uz*ux*(1-c) - uy*s, uz*uy*(1-c) + ux*s, c + uz*uz*(1-c)]
        ])
        T = T @ T_static @ rotation_matrix
        
    return T[:3, 3]

def visualize_workspace(points):
    if points.size == 0:
        print("警告: 没有可供显示的工作空间点。")
        return
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='b', marker='.', s=10, alpha=0.6)
    ax.set_xlabel('X Axis (m)')
    ax.set_ylabel('Y Axis (m)')
    ax.set_zlabel('Z Axis (m)')
    ax.set_title('Robot Arm Workspace Visualization')
    max_range = np.array([points[:, 0].max()-points[:, 0].min(), 
                          points[:, 1].max()-points[:, 1].min(), 
                          points[:, 2].max()-points[:, 2].min()]).max() / 2.0
    mid_x = (points[:, 0].max()+points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max()+points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max()+points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    plt.show()

def main():
    if not os.path.exists(URDF_FILENAME):
        print(f"错误: URDF文件 '{URDF_FILENAME}' 未找到。请确保该文件与脚本在同一目录下。")
        return
    with open(URDF_FILENAME, 'r', encoding='utf-8') as f:
        urdf_content = f.read()
    print("URDF文件读取成功。开始计算...")
    joint_info = parse_urdf_for_kinematics(urdf_content)
    workspace_chain = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
    sample_joints = {j: joint_info[j] for j in workspace_chain if j in joint_info}
    workspace_points = []
    num_samples = 10
    print(f"每个主要关节采样 {num_samples} 个点...")
    joint_angle_ranges = [np.linspace(params['lower'], params['upper'], num_samples) for name, params in sample_joints.items()]
    angle_combinations = np.meshgrid(*joint_angle_ranges)
    flat_combinations = [c.flatten() for c in angle_combinations]
    total_points = len(flat_combinations[0])
    print(f"总共需要计算 {total_points} 个点...")
    for i in range(total_points):
        current_angles = {name: flat_combinations[j][i] for j, name in enumerate(sample_joints)}
        end_effector_pos = forward_kinematics(current_angles, joint_info, workspace_chain)
        workspace_points.append(end_effector_pos)
    workspace_points = np.array(workspace_points)
    print("计算完成。")
    min_coords = np.min(workspace_points, axis=0)
    max_coords = np.max(workspace_points, axis=0)
    print("\n-------------------------------------------")
    print("  机械臂活动范围 (末端腕部)")
    print("-------------------------------------------")
    print(f"  X轴范围: 从 {min_coords[0]:.4f} 到 {max_coords[0]:.4f} 米")
    print(f"  Y轴范围: 从 {min_coords[1]:.4f} 到 {max_coords[1]:.4f} 米")
    print(f"  Z轴范围: 从 {min_coords[2]:.4f} 到 {max_coords[2]:.4f} 米")
    print("-------------------------------------------")
    print("\n正在生成三维可视化图形...")
    visualize_workspace(workspace_points)

if __name__ == '__main__':
    main()