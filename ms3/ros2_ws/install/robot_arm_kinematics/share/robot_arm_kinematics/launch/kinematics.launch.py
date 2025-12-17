"""
Launch file for robot arm kinematics and control
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with kinematics nodes"""
    
    return LaunchDescription([
        # Forward kinematics node
        Node(
            package='robot_arm_kinematics',
            executable='forward_kinematics_node',
            name='forward_kinematics',
            output='screen',
        ),
        
        # Inverse kinematics node
        Node(
            package='robot_arm_kinematics',
            executable='inverse_kinematics_node',
            name='inverse_kinematics',
            output='screen',
        ),
        
        # Trajectory planner node
        Node(
            package='robot_arm_kinematics',
            executable='trajectory_planner_node',
            name='trajectory_planner',
            output='screen',
        ),
    ])
