#!/usr/bin/env python3

import subprocess
import time
import os

# Correct file path
qgc_path = os.path.expanduser("~/QGroundControl-x86_64.AppImage")

# Get the absolute path of pcl_converter.py (assuming it's in the same directory as this script)
script_dir = os.path.dirname(os.path.abspath(__file__))
pcl_converter_path = os.path.join(script_dir, "pcl_converter.py")

# List of commands
commands = [
    "MicroXRCEAgent udp4 -p 8888",
    "cd ~/PX4-Autopilot && PX4_SITL_WORLD=$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/baylands.world make px4_sitl_default gazebo-classic_iris_depth_camera",
    f"{qgc_path}",  # Launch QGroundControl
    "bash -c 'source ~/RRC_Project/Test_ws/install/setup.bash && ros2 launch genz_icp odometry.launch.py topic:=/camera/points'",
    f"bash -c 'cd {script_dir} && python3 pcl_converter.py'"
]

# Run commands in new terminal tabs
for command in commands:
    subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", f"{command}; exec bash"])
    time.sleep(1)