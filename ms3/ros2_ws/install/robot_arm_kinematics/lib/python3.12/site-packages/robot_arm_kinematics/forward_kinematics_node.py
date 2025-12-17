"""
ROS2 Node for Forward Kinematics Service

Provides a ROS2 service to compute forward kinematics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np

from robot_arm_kinematics.forward_kinematics import ForwardKinematics


class ForwardKinematicsNode(Node):
    """ROS2 node that publishes forward kinematics"""
    
    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        # Initialize FK solver
        self.fk = ForwardKinematics()
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publish end-effector pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/end_effector_pose',
            10
        )
        
        self.get_logger().info('Forward Kinematics Node started')
    
    def joint_state_callback(self, msg: JointState):
        """
        Callback for joint state messages
        
        Args:
            msg: JointState message with current joint angles
        """
        # Extract joint angles (assuming order: joint1, joint2, joint3, joint4)
        if len(msg.position) < 4:
            return
        
        joint_angles = list(msg.position[:4])
        
        # Compute forward kinematics
        position, rotation = self.fk.compute_fk(joint_angles)
        
        # Create and publish pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        
        pose_msg.pose.position.x = float(position[0])
        pose_msg.pose.position.y = float(position[1])
        pose_msg.pose.position.z = float(position[2])
        
        # Convert rotation matrix to quaternion (simplified)
        # For full implementation, use transforms3d or scipy
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
