"""
ROS2 Node for Trajectory Planning and Execution

Plans and executes trajectories for the robot arm
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import threading

from robot_arm_kinematics.trajectory_planner import TrajectoryPlanner


class TrajectoryPlannerNode(Node):
    """ROS2 node for trajectory planning"""
    
    def __init__(self):
        super().__init__('trajectory_planner_node')
        
        # Initialize planner
        self.planner = TrajectoryPlanner(dt=0.01)
        
        # Current joint state
        self.current_joints = None
        
        # Subscribe to current joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to goal joint states
        self.goal_sub = self.create_subscription(
            JointState,
            '/desired_joint_states',
            self.goal_callback,
            10
        )
        
        # Publish trajectory points
        self.traj_pub = self.create_publisher(
            JointState,
            '/trajectory_point',
            10
        )
        
        # Timer for trajectory execution
        self.trajectory = None
        self.traj_index = 0
        self.executing = False
        
        self.timer = self.create_timer(0.01, self.execute_trajectory_step)
        
        self.get_logger().info('Trajectory Planner Node started')
    
    def joint_state_callback(self, msg: JointState):
        """Update current joint state"""
        if len(msg.position) >= 4:
            self.current_joints = np.array(msg.position[:4])
    
    def goal_callback(self, msg: JointState):
        """
        Callback for goal joint states - plan trajectory
        
        Args:
            msg: Goal JointState
        """
        if self.current_joints is None:
            self.get_logger().warn('No current joint state available')
            return
        
        goal_joints = np.array(msg.position[:4])
        
        self.get_logger().info(f'Planning trajectory to goal: {goal_joints}')
        
        # Plan trajectory
        self.trajectory = self.planner.smooth_joint_trajectory(
            self.current_joints,
            goal_joints,
            duration=3.0,
            profile='quintic'
        )
        
        self.traj_index = 0
        self.executing = True
        
        self.get_logger().info(f'Trajectory planned with {len(self.trajectory)} points')
    
    def execute_trajectory_step(self):
        """Execute one step of the trajectory"""
        if not self.executing or self.trajectory is None:
            return
        
        if self.traj_index >= len(self.trajectory):
            self.executing = False
            self.get_logger().info('Trajectory execution complete')
            return
        
        # Publish current trajectory point
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        joint_msg.position = self.trajectory[self.traj_index].tolist()
        
        self.traj_pub.publish(joint_msg)
        
        self.traj_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
