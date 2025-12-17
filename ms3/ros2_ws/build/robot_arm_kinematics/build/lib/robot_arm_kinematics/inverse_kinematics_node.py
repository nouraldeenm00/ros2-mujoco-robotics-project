"""
ROS2 Node for Inverse Kinematics Service

Provides a ROS2 service to compute inverse kinematics
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np

from robot_arm_kinematics.inverse_kinematics import InverseKinematics


class InverseKinematicsNode(Node):
    """ROS2 node that provides IK service"""
    
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Initialize IK solver
        self.ik = InverseKinematics()
        
        # Subscribe to target pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.target_pose_callback,
            10
        )
        
        # Publish desired joint states
        self.joint_pub = self.create_publisher(
            JointState,
            '/desired_joint_states',
            10
        )
        
        self.get_logger().info('Inverse Kinematics Node started')
    
    def target_pose_callback(self, msg: PoseStamped):
        """
        Callback for target pose messages
        
        Args:
            msg: PoseStamped message with target end-effector pose
        """
        # Extract target position
        target_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        self.get_logger().info(f'Received target: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]')
        
        # Compute inverse kinematics
        solution = self.ik.compute_ik(target_pos, wrist_angle=0.0)
        
        if solution is not None:
            # Create and publish joint state message
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
            joint_msg.position = solution
            
            self.joint_pub.publish(joint_msg)
            self.get_logger().info(f'IK solution found: {[f"{a:.3f}" for a in solution]}')
        else:
            self.get_logger().warn('No IK solution found for target position')


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
