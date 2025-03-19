import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import threading  # ✅ Add threading

class Arm3DVisualizer(Node):
    def __init__(self):
        super().__init__('arm_3d_visualizer')
        self.subscription = self.create_subscription(
            Float32MultiArray, '/joint_angles', self.update_angles, 10)

        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.arm_lengths = [1.5, 1.2, 0.5]

        self.root = tk.Tk()
        self.root.title("3D Arm Visualizer")

        self.fig = plt.figure(figsize=(6, 6))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack()

        self.update_plot()
        self.root.after(10, self.update_gui)

        # ✅ Start ROS2 listener in a separate thread
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.ros_thread.start()

        self.root.mainloop()

    def update_angles(self, msg):
        self.joint_angles = list(msg.data)
        self.update_plot()

    def forward_kinematics(self):
        base_angle = np.radians(self.joint_angles[0])
        joint_angles = np.radians(self.joint_angles[1:])
        x, y, z = [0], [0], [0]

        for i in range(3):
            x.append(x[-1] + self.arm_lengths[i] * np.cos(base_angle) * np.cos(np.sum(joint_angles[:i+1])))
            y.append(y[-1] + self.arm_lengths[i] * np.sin(base_angle) * np.cos(np.sum(joint_angles[:i+1])))
            z.append(z[-1] + self.arm_lengths[i] * np.sin(np.sum(joint_angles[:i+1])))

        return x, y, z

    def update_plot(self):
        self.ax.clear()
        x, y, z = self.forward_kinematics()
        self.ax.plot(x, y, z, 'ro-', markersize=5, linewidth=2)
        self.ax.set_xlim(-3, 3)
        self.ax.set_ylim(-3, 3)
        self.ax.set_zlim(0, 3)
        self.ax.set_xlabel("X Axis")
        self.ax.set_ylabel("Y Axis")
        self.ax.set_zlabel("Z Axis")
        self.ax.set_title("3D Arm Motion")
        self.ax.grid(True)

        self.canvas.draw()

    def update_gui(self):
        self.update_plot()
        self.root.after(10, self.update_gui)

def main(args=None):
    rclpy.init(args=args)
    visualizer = Arm3DVisualizer()
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
