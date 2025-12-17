"""
Launch file for integrated robot demo with RViz2 visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for complete system"""
    
    # Get package directories
    robot_description_share = FindPackageShare('robot_arm_description')
    
    # RViz config
    rviz_config_file = PathJoinSubstitution(
        [robot_description_share, 'rviz', 'display.rviz']
    )
    
    return LaunchDescription([
        # Robot state publisher (from MS2)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': open('/home/nour/ros2-mujoco-robotics-project/ms2/ros2_ws/src/robot_arm_description/urdf/robot_arm.urdf.xacro').read()
            }],
            output='screen',
        ),
        
        # RViz2 visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        ),
        
        # Integrated demo node
        ExecuteProcess(
            cmd=['python3', '/home/nour/ros2-mujoco-robotics-project/ms3/demos/integrated_demo.py'],
            output='screen',
        ),
    ])
