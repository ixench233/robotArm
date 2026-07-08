# robot_control/kinematics.py
import numpy as np
import os
import tempfile
from ikpy import chain
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RobotKinematics:
    def __init__(self, urdf_file_path, workspace_limits):
        with open(urdf_file_path, 'r', encoding='utf-8') as f:
            urdf_content = f.read()
        modified_content = urdf_content.replace('type="continuous"', 'type="revolute"')
        try:
            with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.urdf', encoding='utf-8') as temp_f:
                temp_f.write(modified_content)
                temp_urdf_path = temp_f.name
            active_mask = [False, True, True, True, True, True, False, False]
            self.my_chain = chain.Chain.from_urdf_file(temp_urdf_path, active_links_mask=active_mask)
        finally:
            if 'temp_urdf_path' in locals() and os.path.exists(temp_urdf_path):
                os.remove(temp_urdf_path)
        self.limits = workspace_limits
        self.fig = plt.figure("Dofbot Simulation")
        self.ax = self.fig.add_subplot(111, projection='3d')
        self._setup_plot()
        plt.ion()
        self.fig.show()

    def _setup_plot(self):
        self.ax.set_xlabel("X (m)"), self.ax.set_ylabel("Y (m)"), self.ax.set_zlabel("Z (m)")
        self.ax.set_xlim([-0.4, 0.4]), self.ax.set_ylim([-0.1, 0.5]), self.ax.set_zlim([0.0, 0.6])
        self.ax.view_init(elev=20, azim=-120)

    def _draw_target_plane(self, z_level):
        xmin, xmax = self.limits['x']
        ymin, ymax = self.limits['y']
        points = np.array([[xmin, ymin, z_level], [xmax, ymin, z_level], [xmax, ymax, z_level], [xmin, ymax, z_level]])
        self.ax.plot(points[[0,1,2,3,0], 0], points[[0,1,2,3,0], 1], z_level, color="g", linestyle="--")

    def solve_ik(self, target_position, initial_angles):
        try:
            return self.my_chain.inverse_kinematics(target_position, initial_position=initial_angles), True
        except Exception:
            return initial_angles, False

    def update_plot(self, joint_angles, target_position):
        self.ax.clear()
        self._setup_plot()
        self._draw_target_plane(target_position[2])
        self.my_chain.plot(joint_angles, self.ax)
        self.ax.scatter(target_position[0], target_position[1], target_position[2], c='red', s=100, label='Target (Wrist)')
        self.ax.legend()
        plt.pause(0.001)

    def close(self):
        plt.close(self.fig)