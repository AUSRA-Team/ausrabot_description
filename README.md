# AUSRA bot description

## Introduction

Welcome to the **ausrabot_description** package. This package contains the URDF, meshes, and configuration files required to simulate the **Ausrabot** omnidirectional mobile robot.

This simulation is built for the **ROS 2 Humble Hawksbill** distribution and utilizes **Gazebo Fortress** for the physics simulation environment. It leverages `ros2_control` and `gz_ros2_control` to provide realistic joint actuation and sensor feedback.

---

## Prerequisites & Installation

Before cloning the repository, ensure you have the necessary dependencies installed on your system (Ubuntu 22.04).

### 1. Install Required Packages

Open your terminal and install the specific control and visualization packages required for this simulation:

```bash
sudo apt update
sudo apt install ros-humble-gz-ros2-control \
ros-humble-ros2-control \
ros-humble-ros2-controllers \
ros-humble-joint-state-publisher-gui
```

## Installing

### Installing from source

> **ATTENTION:** These commands assume that you have created a workspace called "ros_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

After installing ROS2 and creating the workspace, clone this repository in your workspace:

```bash
cd ~/ros_ws/src
git clone https://github.com/mateusmenezes95/axebot.git
```

Now, install the omnidirectional controller package by running:

```bash
cd ~/ros_ws/src
git clone https://github.com/mateusmenezes95/omnidirectional_controllers.git
```

## Building

Run the following command to build and then source the package:

```bash
cd ~/ros_ws
colcon build --symlink-install
```

To launch the Gazebo simulation and spawn the robot into it, run the following command:

```bash
ros2 launch ausrabot_description ausrabot_.launch.py
```

To move the robot in a straight line with a velocity of 0.5 m/s in the x-axis, execute the following command:

```bash
ros2 topic pub \
omnidirectional_controller/cmd_vel_unstamped \
-r 10 \
geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Visualizing Lidar Data

### In Gazebo Fortress

You can visualize the Lidar rays directly within the Gazebo simulation to verify the sensor is working.

1. Click on the triple dot icon (plugin menu) in the top-right corner of the Gazebo window.
2. Search for **“Visualize Lidar”**.
3. A new block will appear in the right-side menu.
4. Click the **Reload/Refresh** button in that block.
5. Select the **/scan** topic from the dropdown list.

**Result:** You should see the blue lidar rays in the simulation window.

---

### In RViz2

To visualize the robot model and actual laser scan data in ROS 2, run the display launch file in a separate terminal:

```bash
ros2 launch ausrabot_description display.launch.py
```

**Testing the Scan:**
Insert a **Cylinder** into the Gazebo world and place it within range of the robot's Lidar.

**Result:** In RViz2, you will see the red laser scan points mapping the surface of the cylinder.

---

If you want, I can export this as a clean downloadable README.md file.

