# Use ROS2 Humble as the base image
FROM ros:humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-rclpy \
    ros-humble-ros2launch \
    python3-tk \
    portaudio19-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    xvfb \
    && rm -rf /var/lib/apt/lists/*  # Cleanup to reduce image size

# Set up the workspace
WORKDIR /my_ws

# Copy the package source code
COPY . /my_ws

# Check if requirements.txt exists before installing
RUN if [ -f "requirements.txt" ]; then \
      grep -v "rclpy" requirements.txt > temp_requirements.txt && \
      pip3 install --no-cache-dir -r temp_requirements.txt; \
    fi

# Ensure ROS 2 environment is sourced
SHELL ["/bin/bash", "-c"]

# Build the ROS 2 package
RUN source /opt/ros/humble/setup.bash && colcon build

# Source the environment when container starts
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /my_ws/install/setup.bash" >> /root/.bashrc

# Default command when container starts
CMD ["/bin/bash"]
