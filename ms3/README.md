# Milestone 3: Kinematics and Simulation

## Overview
This milestone implements forward/inverse kinematics, trajectory planning, and MuJoCo simulation for the 4-DOF robot arm.

## Components

### 1. Kinematics (`ros2_ws/src/robot_arm_kinematics/`)
- **Forward Kinematics**: Computes end-effector position from joint angles
- **Inverse Kinematics**: Computes joint angles from desired end-effector position
- **Trajectory Planning**: Generates smooth trajectories between waypoints
- **ROS2 Nodes**: Integration with ROS2 for real-time control

### 2. MuJoCo Simulation (`mujoco/`)
- **Robot Model**: MuJoCo XML description of the 4-DOF arm
- **Simulator Interface**: Python interface for control and visualization
- **Position Control**: Actuators for joint-level control

### 3. Demo Scripts (`demos/`)
- **test_kinematics.py**: Test FK/IK without simulation
- **test_mujoco.py**: MuJoCo simulation demos

## Installation

### Prerequisites
```bash
pip install mujoco numpy scipy
```

### Build ROS2 Package
```bash
cd ~/ros2-mujoco-robotics-project/ms3/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_kinematics
source install/setup.bash
```

## Usage

### Test Kinematics (Standalone)
```bash
cd ~/ros2-mujoco-robotics-project/ms3/demos
python3 test_kinematics.py
```

### Test MuJoCo Simulation

**Basic Interactive Mode:**
```bash
python3 test_mujoco.py --mode basic
```

**Predefined Trajectory:**
```bash
python3 test_mujoco.py --mode trajectory
```

**IK-based Control:**
```bash
python3 test_mujoco.py --mode ik
```

### Run ROS2 Nodes
```bash
# Launch all kinematics nodes
ros2 launch robot_arm_kinematics kinematics.launch.py

# In another terminal, send a target pose
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'world'},
  pose: {
    position: {x: 0.3, y: 0.0, z: 0.5}
  }
}"
```

## Features

### Forward Kinematics
- Geometric transformation approach
- Homogeneous transformation matrices
- Position and orientation output
- Jacobian computation (for velocity kinematics)

### Inverse Kinematics
- Geometric analytical solution
- Numerical optimization fallback (SLSQP)
- Joint limit constraints
- Multiple solution handling (elbow up/down)

### Trajectory Planning
- Linear interpolation in joint space
- Smooth profiles: cubic, quintic, trapezoidal
- Multi-waypoint trajectories
- Configurable duration and time step

### MuJoCo Simulation
- Real-time visualization
- Position-controlled actuators
- Joint position/velocity sensors
- End-effector position tracking
- Interactive GUI control

## Architecture

```
ms3/
├── ros2_ws/src/robot_arm_kinematics/    # ROS2 package
│   ├── robot_arm_kinematics/
│   │   ├── forward_kinematics.py        # FK implementation
│   │   ├── inverse_kinematics.py        # IK solver
│   │   ├── trajectory_planner.py        # Trajectory generation
│   │   ├── forward_kinematics_node.py   # FK ROS2 node
│   │   ├── inverse_kinematics_node.py   # IK ROS2 node
│   │   └── trajectory_planner_node.py   # Planner ROS2 node
│   ├── launch/
│   │   └── kinematics.launch.py         # Launch file
│   ├── package.xml
│   └── setup.py
├── mujoco/
│   ├── robot_arm.xml                    # MuJoCo model
│   └── mujoco_simulator.py              # Simulator interface
└── demos/
    ├── test_kinematics.py               # Kinematics tests
    └── test_mujoco.py                   # Simulation demos
```

## ROS2 Topics

### Published
- `/end_effector_pose` (geometry_msgs/PoseStamped): Current EE pose from FK
- `/desired_joint_states` (sensor_msgs/JointState): IK solution
- `/trajectory_point` (sensor_msgs/JointState): Trajectory points

### Subscribed
- `/joint_states` (sensor_msgs/JointState): Current joint angles
- `/target_pose` (geometry_msgs/PoseStamped): Target EE pose for IK

## Testing

All components include standalone test functions. Run individual modules:

```bash
# Test FK
python3 -c "from robot_arm_kinematics.forward_kinematics import test_forward_kinematics; test_forward_kinematics()"

# Test IK
python3 -c "from robot_arm_kinematics.inverse_kinematics import test_inverse_kinematics; test_inverse_kinematics()"

# Test trajectory planner
python3 -c "from robot_arm_kinematics.trajectory_planner import test_trajectory_planner; test_trajectory_planner()"
```

## Next Steps (Milestone 4)
- Advanced control algorithms (PID, computed torque)
- Dynamic simulation and contact handling
- Task-space control
- Path planning with obstacle avoidance
- ROS2-MuJoCo full integration
