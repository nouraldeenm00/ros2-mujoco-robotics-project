# Quick Start Guide

## Prerequisites
```bash
# Ubuntu 24.04 with ROS2 Jazzy
sudo apt install ros-jazzy-desktop ros-jazzy-joint-state-publisher-gui
pip install numpy scipy
```

## Installation
```bash
cd ~/ros2-mujoco-robotics-project

# Build MS2 (URDF and visualization)
cd ms2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_description
source install/setup.bash

# Build MS3 (Kinematics and control)
cd ../../ms3/ros2_ws
colcon build --packages-select robot_arm_kinematics
source install/setup.bash
```

## Quick Tests

### Test 1: Kinematics Validation
```bash
cd ~/ros2-mujoco-robotics-project/ms3/demos
python3 validate_kinematics.py
```
**Expected**: All tests pass ✓

### Test 2: RViz2 Visualization
```bash
cd ~/ros2-mujoco-robotics-project/ms2/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```
**Expected**: RViz2 opens with robot model and joint sliders

### Test 3: Complete Integrated Demo
```bash
# Terminal 1
cd ~/ros2-mujoco-robotics-project/ms2/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch robot_arm_description display.launch.py

# Terminal 2
cd ~/ros2-mujoco-robotics-project/ms3/demos
python3 integrated_demo.py
```
**Expected**: Robot executes autonomous trajectory sequence

## Commands Summary

| Task | Command |
|------|---------|
| Test kinematics | `cd ms3/demos && python3 test_kinematics.py` |
| Validate all | `cd ms3/demos && python3 validate_kinematics.py` |
| Visualize robot | `ros2 launch robot_arm_description display.launch.py` |
| Run full demo | `python3 ms3/demos/integrated_demo.py` |

## Troubleshooting

**Issue**: `ModuleNotFoundError: No module named 'catkin_pkg'`
**Fix**: `pip install catkin_pkg`

**Issue**: RViz2 doesn't show robot
**Fix**: Check that `robot_state_publisher` is running

**Issue**: OpenGL/MuJoCo errors
**Solution**: Use RViz2 instead (already configured)
