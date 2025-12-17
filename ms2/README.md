# Milestone 2: URDF Model and ROS2 Simulation

## Overview
This milestone contains the URDF model of the 4-DOF robotic arm and ROS2 visualization setup.

## Project Structure
```
ms2/
├── ros2_ws/               # ROS2 workspace
│   └── src/
│       └── robot_arm_description/
│           ├── urdf/      # URDF/XACRO files
│           ├── meshes/    # 3D mesh files (STL/DAE)
│           ├── launch/    # Launch files
│           ├── rviz/      # RViz configuration
│           └── config/    # Configuration files
```

## Robot Specifications
- **DOF**: 4 (Degrees of Freedom)
- **Joints**:
  - Joint 1: Base rotation (revolute, Z-axis, ±180°)
  - Joint 2: Shoulder (revolute, Y-axis, ±90°)
  - Joint 3: Elbow (revolute, Y-axis, ±90°)
  - Joint 4: Wrist rotation (revolute, Z-axis, ±180°)

## Setup Instructions

### Prerequisites
- ROS2 Jazzy (Ubuntu 24.04)
- colcon build tools

### Installation

1. **Install ROS2 Jazzy** (if not already installed):
```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-joint-state-publisher-gui
```

2. **Build the workspace**:
```bash
cd ~/ros2-mujoco-robotics-project/ms2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_description
source install/setup.bash
```

### Running the Visualization

Launch RViz2 with the robot model:
```bash
source ~/ros2-mujoco-robotics-project/ms2/ros2_ws/install/setup.bash
ros2 launch robot_arm_description display.launch.py
```

This will open:
- **RViz2**: 3D visualization of the robot
- **Joint State Publisher GUI**: Interactive sliders to control each joint

## Usage

1. Use the Joint State Publisher GUI sliders to move each joint
2. Observe the robot movement in RViz2
3. The TF frames show the coordinate systems for each link

## Files Description

- `urdf/robot_arm.urdf.xacro`: Main URDF description with parametric design
- `launch/display.launch.py`: Launch file for visualization
- `rviz/display.rviz`: RViz configuration
- `package.xml`: ROS2 package dependencies
- `CMakeLists.txt`: Build configuration

## Next Steps (Milestone 3)
- Implement forward/inverse kinematics
- Add trajectory planning
- Integrate with MuJoCo simulator
- Develop control algorithms
