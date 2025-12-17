#!/bin/bash

# Complete visualization launcher with object and flags
# This runs everything you need to see the robot moving with visual markers

echo "🤖 Starting Robot Arm Visualization with Object & Flags"
echo "========================================================"

# Kill any existing processes
killall -9 rviz2 joint_state_publisher_gui robot_state_publisher python3 2>/dev/null
sleep 1

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Start RViz2 and robot in background (NO joint_state_publisher_gui to avoid flickering)
echo "1️⃣  Launching RViz2 with robot model..."
cd ~/ros2-mujoco-robotics-project/ms2/ros2_ws
source install/setup.bash
ros2 launch robot_arm_description motion.launch.py &
RVIZ_PID=$!

# Wait for RViz2 to start
sleep 5
echo "2️⃣  RViz2 launched (PID: $RVIZ_PID)"
echo "   Checking if RViz2 window is visible..."
if pgrep -x rviz2 > /dev/null; then
    echo "   ✅ RViz2 process running"
else
    echo "   ❌ RViz2 not running!"
    exit 1
fi

# Start scene markers (object and flags)
echo "3️⃣  Starting scene markers (object & flags)..."
source /opt/ros/jazzy/setup.bash
cd ~/ros2-mujoco-robotics-project/ms3/ros2_ws
source install/setup.bash
/usr/bin/python3 src/robot_arm_kinematics/scripts/scene_markers.py &
MARKERS_PID=$!
sleep 2
echo "   Scene markers running (PID: $MARKERS_PID)"
echo "   Verifying /scene_markers topic..."
if ros2 topic list | grep -q scene_markers; then
    echo "   ✅ Markers topic publishing"
else
    echo "   ❌ Markers topic not found!"
fi

# Start motion visualizer
echo "4️⃣  Starting robot motion sequence..."
echo ""
echo "   You should now see in RViz2:"
echo "   ✅ 4-DOF robot arm"
echo "   ✅ Yellow box (object to pick)"
echo "   ✅ Green flag (start/pick location)"
echo "   ✅ Red flag (end/place location)"
echo ""
echo "   The robot will now perform:"
echo "   - Motion 1: Reach to pick object"
echo "   - Motion 2: Circular assembly motion"
echo "   - Motion 3: Pick and place sequence"
echo ""
echo "========================================================"
echo ""

/usr/bin/python3 src/robot_arm_kinematics/scripts/motion_visualizer.py

# RViz2 and markers stay running for you to explore
echo ""
echo "========================================================"
echo "🎉 Demonstration complete! RViz2 is still running."
echo "   Close the RViz2 window to exit, or press Ctrl+C here."
echo "========================================================"

# Keep waiting until user closes RViz2 window or presses Ctrl+C
wait $RVIZ_PID 2>/dev/null

# Cleanup on exit
echo ""
echo "Cleaning up..."
kill $MARKERS_PID 2>/dev/null
killall -9 python3 rviz2 joint_state_publisher_gui robot_state_publisher 2>/dev/null
echo "Done!"
