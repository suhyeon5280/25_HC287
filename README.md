![Logo](readme/logo.png)
# PUPILOT

### Prebuild environment
* OS  (Ubuntu 22.04 LTS)  
* ROS Environment (ROS2 Humble)
* Python 3.10 (system)

### Installation
1. sudo apt-get update && sudo apt-get install -y git build-essential python3-pip python3-venv \
    ros-humble-desktop ros-humble-colcon-common-extensions
2. git clone https://github.com/suhyeon5280/25_HC287.git

### IMU Configuration
Install the sensor connect, and set the configuration as below.
![dependency](readme/dependency.png)

### Build examples
1. Put this package in your workspace.
2. colcon build --symlink-install
3. source your/workspace/install/setup.bash
4. Lanch :
   LiDAR: ros2 launch rplidar_ros rplidar.launch.py serial_baud:=115200
   SLAM: ros2 launch slam_toolbox online_async_launch.py
   (Optional) App: cd ~/espers2_ws/src/25_HC287/app && source ../.venv/bin/activate && python3 camera_server.py
5. The topics for IMU informatoins are available.
