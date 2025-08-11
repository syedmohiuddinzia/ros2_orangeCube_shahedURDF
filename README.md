# Using Shahed URDF in ROS2 for Orange Cube IMU Data
This guide explains how to use the Shahed URDF in ROS 2 to represent and visualize Orange Cube IMU data. This repository contains the Shahed UAV URDF for use with ROS 2. You can visualize the model in RViz and integrate it with your existing ROS 2 workspace.

---

### Note:
This project builds upon the foundational setup detailed in [ros2_orangeCube_integration](https://github.com/syedmohiuddinzia/ros2_orangeCube_integration), and [ros2_orangeCube_imuMadgwickFilter](https://github.com/syedmohiuddinzia/ros2_orangeCube_imuMadgwickFilter)
Please make sure to follow the integration steps in the first repository to get your Cube Orange connected with ROS 2 and MAVROS properly, and then complete the second repository to ensure your IMU data is filtered and oriented before using it in this project. 

---
### Package Structure
```
shahed/
├── launch/
│   └── view_model.launch.py      # RViz launch file
├── urdf/
│   └── shahed.urdf               # UAV URDF description
├── CMakeLists.txt
└── package.xml
```
---

## What is URDF?
A URDF (Unified Robot Description Format) is basically a text file in XML format that describes your robot’s physical structure so ROS can visualize, simulate, and understand it.
Think of it as your robot’s blueprint for the digital world — it tells ROS things like:
Main things URDF describes:
- **Links** *→ rigid parts of the robot (e.g., chassis, arm segments, wheels).*
- **Joints** *→ how the links are connected (e.g., fixed, revolute, continuous).*
- **Geometry** *→ shape of each link (box, cylinder, mesh).*
- **Visuals** *→ what it looks like in RViz (color, texture, mesh files).*
- **Collisions** *→ simplified shapes used for physics simulation.*
- **Inertial properties** *→ mass, center of gravity, moments of inertia.*

---

## Prerequisites
Make sure you have:
- ROS 2 Humble (or your desired ROS 2 distribution) installed
- colcon build tool installed
- A ROS 2 workspace already created (~/ros2_ws)
- rviz2 installed

**Clone the Repository**
Navigate to the src folder of your ROS 2 workspace and clone the repo:
```bash
cd ~/ros2_ws/src
git clone https://github.com/syedmohiuddinzia/ros2_orangeCube_shahedURDF.git
```

**Build the Package**
After cloning, build the workspace:
```bash
cd ~/ros2_ws
colcon build
```

**Source the setup file so ROS 2 can find the package:**
```bash
source install/setup.bash
```
Note: You can add this line to your `~/.bashrc` so it runs automatically each time you open a terminal:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
---

### View in RViz2
Launch the model visualization:
```bash
ros2 launch shahed view_model.launch.py
```
In RVIZ2
- Change **Fixed Frame** to `base_link`.
- Add a **RobotModel** display.
- In the **Description Topic** field of the RobotModel display, choose `/robot_description`.


After running the launch file, RViz should open showing the Shahed UAV 3D model with correct joints and links.

