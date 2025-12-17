import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/nour/ros2-mujoco-robotics-project/ms3/ros2_ws/install/robot_arm_kinematics'
