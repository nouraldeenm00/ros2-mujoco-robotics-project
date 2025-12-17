# Pick and Place Demonstration

## Overview
The robot arm now performs a complete industrial pick-and-place operation with proper object manipulation physics.

## What You See in RViz2

### Scene Elements
1. **Yellow Box** - The object to be manipulated (8cm cube)
2. **Green Flag** - Pick location at `[0.35, 0.1, 0.65]`
3. **Red Flag** - Place location at `[0.3, -0.15, 0.65]`
4. **Robot Arm** - 4-DOF manipulator with real-time joint state updates

### Motion Sequence (10 Steps)

#### 1️⃣ Home Position
- Robot starts at all joints = 0°
- Gripper OPEN
- Object at green flag

#### 2️⃣ Move Above Pick
- Robot moves to position 10cm above green flag
- Duration: 4.0 seconds
- Smooth trajectory planning

#### 3️⃣ Lower to Pick
- Robot descends to pick location (green flag height)
- Duration: 2.0 seconds
- Gripper still OPEN

#### 4️⃣ **GRASP** Object
- **Gripper closes** (`/gripper_state` = True)
- Object attaches to robot end effector
- Pause: 1.0 second

#### 5️⃣ Lift Object
- Robot lifts back to 10cm above pick
- Object moves with robot (attached)
- Duration: 2.0 seconds

#### 6️⃣ Move to Place
- Robot travels to position above red flag
- Object follows robot motion
- Duration: 4.0 seconds

#### 7️⃣ Lower to Place
- Robot descends to place location (red flag height)
- Object still attached
- Duration: 2.0 seconds

#### 8️⃣ **RELEASE** Object
- **Gripper opens** (`/gripper_state` = False)
- Object detaches and remains at place location
- Pause: 1.0 second

#### 9️⃣ Lift Up
- Robot rises 10cm above place location
- Object stays at red flag
- Duration: 2.0 seconds

#### 🔟 Return Home
- Robot returns to home position
- Object remains at red flag
- Duration: 4.0 seconds

**Total Duration:** ~25-30 seconds

## Object Physics

### Object Tracking Logic
The object behavior is controlled by `scene_markers.py`:

```python
def joint_callback(self, msg):
    # Track end effector position from joint states
    joints = np.array(msg.position[:4])
    pos, _ = self.fk.compute_fk(joints)
    self.end_effector_pos = pos.flatten().tolist()
    
    # If gripper closed, object follows end effector
    if self.gripper_closed:
        self.object_pos = [
            self.end_effector_pos[0],
            self.end_effector_pos[1],
            self.end_effector_pos[2] - 0.05  # 5cm below end effector
        ]
```

### Gripper Control
The gripper state is published by `motion_visualizer.py`:

```python
def publish_gripper_state(self, closed):
    msg = Bool()
    msg.data = closed
    self.gripper_pub.publish(msg)  # Topic: /gripper_state
```

## Technical Implementation

### ROS2 Topics
- `/joint_states` - Robot joint positions (50 Hz)
- `/scene_markers` - Visual markers for RViz2 (20 Hz)
- `/gripper_state` - Boolean (True=closed, False=open)
- `/robot_description` - URDF model

### Key Components
1. **motion_visualizer.py** - Publishes joint states and gripper control
2. **scene_markers.py** - Publishes object/flag markers, handles object attachment
3. **motion.launch.py** - Launches RViz2 without GUI to avoid flickering
4. **motion.rviz** - RViz2 configuration with MarkerArray display

### Coordinate System
- **Frame:** `world` (base of robot)
- **Units:** Meters
- **Workspace:** Reachable zone within robot's kinematic limits

## Running the Demonstration

### Single Command
```bash
cd ~/ros2-mujoco-robotics-project
./run_visualization.sh
```

### Manual Steps
```bash
# Terminal 1: Launch RViz2
source ~/ros2-mujoco-robotics-project/ms2/ros2_ws/install/setup.bash
ros2 launch robot_arm_description motion.launch.py

# Terminal 2: Start scene markers
source ~/ros2-mujoco-robotics-project/ms3/ros2_ws/install/setup.bash
python3 ~/ros2-mujoco-robotics-project/ms3/ros2_ws/src/robot_arm_kinematics/scripts/scene_markers.py

# Terminal 3: Run motion sequence
source ~/ros2-mujoco-robotics-project/ms3/ros2_ws/install/setup.bash
python3 ~/ros2-mujoco-robotics-project/ms3/ros2_ws/src/robot_arm_kinematics/scripts/motion_visualizer.py
```

## Expected Behavior

✅ **Object starts at green flag** (pick location)
✅ **Object stays at green flag** until gripper closes (step 4)
✅ **Object follows robot** from step 4 to step 8 (while gripper closed)
✅ **Object stays at red flag** after release (step 8 onwards)
✅ **Smooth motion** with trajectory planning (no jumps)
✅ **Proper gripper timing** (1s pause at grasp/release)

## Troubleshooting

### Object doesn't move
- Check `/gripper_state` topic: `ros2 topic echo /gripper_state`
- Verify scene_markers is subscribed: `ros2 node info /scene_marker_publisher`

### Motion fails
- Positions may be outside workspace
- Check IK solutions with `test_kinematics.py`

### Flickering
- Ensure only `motion.launch.py` is used (not `display.launch.py`)
- No `joint_state_publisher_gui` should be running

## Performance Metrics

- **IK Accuracy:** 0.001mm (validated with 30/30 tests)
- **Trajectory Smoothness:** Joint space quintic polynomials
- **Update Rate:** 50 Hz joint states, 20 Hz markers
- **Response Time:** < 20ms for FK/IK computations

## Files Modified for This Demo

1. `/ms3/ros2_ws/src/robot_arm_kinematics/scripts/motion_visualizer.py`
   - Added gripper publisher
   - Implemented 10-step pick-and-place sequence
   - Added smooth_move() helper function

2. `/ms3/ros2_ws/src/robot_arm_kinematics/scripts/scene_markers.py`
   - Added joint_states subscriber
   - Added gripper_state subscriber
   - Implemented object attachment logic

3. `/ms2/ros2_ws/src/robot_arm_description/launch/motion.launch.py`
   - Removed joint_state_publisher_gui to fix flickering
   - Uses xacro for URDF loading

4. `/ms2/ros2_ws/src/robot_arm_description/rviz/motion.rviz`
   - Added MarkerArray display for /scene_markers
   - Configured camera view for optimal viewing

5. `/run_visualization.sh`
   - Single-command launcher with diagnostics
   - Automatic process management and verification

## Next Steps

- ✅ Complete MS1-MS5 implementation
- ✅ All validation tests passing (30/30)
- ✅ Pick-and-place visualization working
- 📹 **Record demonstration video for submission**
- 📄 **Complete final project report**

---
**Status:** ✅ **WORKING**  
**Last Updated:** December 16, 2025  
**Course:** MCTR911 - Robotics Systems
