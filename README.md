# Robot Arm Visualizer

A ROS2-based GUI application for visualizing and controlling a robotic arm in real time using Tkinter and Matplotlib.

## Features
- **Real-time Joint Angle Visualization**: Updates in response to ROS2 messages.
- **3D Visualization**: Displays the robot arm's movement in 3D using Matplotlib.
- **GUI-based Control**: Adjustable sliders to manually control joint angles.
- **ROS2 Integration**: Subscribes and publishes to the `/joint_angles` topic for synchronization.

## Prerequisites
- ROS2 Humble installed
- Python 3.x
- Required dependencies:
  ```bash
  pip install numpy matplotlib rclpy tkinter
  ```

## Installation & Usage
### Clone the Repository
```bash
git clone https://github.com/abulhasnat-abdullah/robot_arm_visualizer.git
cd arm_visualizer
```

### Build and Source the ROS2 Workspace
```bash
cd ~/gui_ws/
colcon build
source install/setup.bash
```

### Run the Visualizer Nodes
#### Start the Joint Angle GUI
```bash
ros2 run arm_visualizer joint_visualizer
```

#### Start the 3D Visualizer
```bash
ros2 run arm_visualizer arm_3d_visualizer
```
## 🐳 Running with Docker
DockerHub : [https://hub.docker.com/repository/docker/abulhasnatabdullah/arm_visualizer]
### **Pull the Docker Image**
```bash
docker pull abulhasnatabdullah/arm_visualizer
```
### **Allow Docker to use your display**
```bash
xhost +local:docker
```
### **Run the Container**
```bash
docker run -it \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
  abulhasnatabdullah/arm_visualizer
```
## Topics
| Topic Name      | Type                    | Description                              |
|---------------|----------------------|----------------------------------|
| `/joint_angles` | `std_msgs/Float32MultiArray` | Publishes/subscribes joint angles |

## Contributing
Feel free to open issues or submit pull requests to enhance the project.

## License
This project is licensed under the MIT License. See `LICENSE` for details.

## Author
Developed by [Abul Hasnat Abdullah](https://github.com/abulhasnat-abdullah).
