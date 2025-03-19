import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class JointVisualizer(Node):
    def __init__(self):
        super().__init__('joint_visualizer')
        self.subscription = self.create_subscription(
            Float32MultiArray, '/joint_angles', self.update_angles, 10)
        self.publisher = self.create_publisher(Float32MultiArray, '/joint_angles', 10)

        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.user_adjusting_slider = False  # Flag to prevent feedback loops

        self.root = tk.Tk()
        self.root.title("Robot Arm Visualizer")
        self.root.configure(bg='grey')

        self.fig, (self.ax_top, self.ax_front) = plt.subplots(1, 2, figsize=(10, 5))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack()

        # Frame for sliders
        slider_frame = tk.Frame(self.root, bg='grey')
        slider_frame.pack(pady=10)

        # Create Sliders for Joint Angles
        self.sliders = []
        for i, label in enumerate(["Base (Top View)", "Shoulder", "Elbow", "Wrist"]):
            frame = tk.Frame(slider_frame, bg='grey')
            frame.pack(side=tk.LEFT, padx=10)
            lbl = tk.Label(frame, text=label, fg='white', bg='grey')
            lbl.pack()
            slider = tk.Scale(frame, from_=-180, to=180, orient="horizontal",
                              command=self.slider_update, bg='#333366', fg='white', highlightbackground='grey')
            slider.bind("<ButtonPress-1>", self.start_slider_adjust)
            slider.bind("<ButtonRelease-1>", self.end_slider_adjust)
            slider.pack()
            self.sliders.append(slider)

        self.update_plot()
        self.root.after(50, self.update_gui)
        self.root.mainloop()

    def update_angles(self, msg):
        """Callback to update joint angles from ROS2 topic."""
        self.joint_angles = list(msg.data)
        self.update_sliders()
        self.update_plot()

    def start_slider_adjust(self, event):
        """User started moving the slider, ignore ROS updates."""
        self.user_adjusting_slider = True

    def end_slider_adjust(self, event):
        """User finished moving the slider, allow ROS updates and publish."""
        self.user_adjusting_slider = False
        self.slider_update(None)  # Publish values once after release

    def slider_update(self, _):
        """Callback when a slider is moved, publishing only after release."""
        if self.user_adjusting_slider:
            return  # Avoid constant publishing while moving sliders
        
        msg = Float32MultiArray()
        msg.data = [float(slider.get()) for slider in self.sliders]
        self.publisher.publish(msg)
        self.joint_angles = msg.data
        self.update_plot()

    def update_sliders(self):
        """Update sliders only if needed (avoid redundant updates)."""
        if self.user_adjusting_slider:
            return  # Ignore updates during user interaction

        for i, angle in enumerate(self.joint_angles):
            if abs(self.sliders[i].get() - angle) > 1e-2:  # Avoid unnecessary updates
                self.sliders[i].set(angle)

    def update_plot(self):
        """Update the Matplotlib visualization."""
        self.ax_top.clear()
        self.ax_front.clear()
        self.ax_top.set_facecolor('LightSteelBlue')
        self.ax_front.set_facecolor('LightSteelBlue')
        self.ax_top.spines['bottom'].set_color('white')
        self.ax_top.spines['left'].set_color('white')
        self.ax_front.spines['bottom'].set_color('white')
        self.ax_front.spines['left'].set_color('white')

        # Convert degrees to radians
        base_angle = np.radians(self.joint_angles[0])  # Base rotation (Z-axis)
        angles = np.radians(self.joint_angles[1:])  # Shoulder, Elbow, Wrist
        
        # Arm segment lengths
        L = [1.5, 1.2, 0.5]

        # **Top View (XY Plane)**
        base_x = np.cos(base_angle)
        base_y = np.sin(base_angle)
        self.ax_top.plot([0, base_x], [0, base_y], 'o-', linewidth=2, markersize=5, color='Indigo')
        self.ax_top.set_xlim(-1.5, 1.5)
        self.ax_top.set_ylim(-1.5, 1.5)
        self.ax_top.set_title("Top View (XY) - Base Rotation", color='Indigo')
        self.ax_top.grid(True, color='black')

        # **Front View (XZ Plane) - Forward Kinematics**
        x, z = [0], [0]  # Starting point at (0,0)
        for i in range(3):
            x.append(x[-1] + L[i] * np.cos(np.sum(angles[:i+1])))
            z.append(z[-1] + L[i] * np.sin(np.sum(angles[:i+1])))

        self.ax_front.plot(x, z, 'bo-', linewidth=2, markersize=5)
        self.ax_front.set_xlim(-3, 3)
        self.ax_front.set_ylim(-3, 3)
        self.ax_front.set_title("Front View (XZ) - Arm Motion", color='Indigo')
        self.ax_front.grid(True, color='black')

        self.canvas.draw()

    def update_gui(self):
        """Ensures real-time updates in Tkinter event loop."""
        self.update_plot()
        self.root.after(50, self.update_gui)

def main(args=None):
    rclpy.init(args=args)
    visualizer = JointVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
