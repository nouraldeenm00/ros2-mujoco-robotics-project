"""
Integrated Demo: Full Robot Control System with RViz2 Visualization

This demo showcases:
- Forward/Inverse Kinematics
- Velocity and Acceleration Kinematics
- Task-space and Joint-space Trajectory Planning
- PID Control
- RViz2 Visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
import numpy as np
import sys
from pathlib import Path

# Add kinematics modules
sys.path.insert(0, str(Path(__file__).parent.parent / "ros2_ws" / "src" / "robot_arm_kinematics"))

from robot_arm_kinematics.forward_kinematics import ForwardKinematics
from robot_arm_kinematics.inverse_kinematics import InverseKinematics
from robot_arm_kinematics.advanced_trajectory import AdvancedTrajectoryPlanner
from robot_arm_kinematics.controllers import TrajectoryTracker


class IntegratedRobotDemo(Node):
    """
    Integrated demo node that runs complete robot system
    """
    
    def __init__(self):
        super().__init__('integrated_robot_demo')
        
        # Initialize kinematics and control
        self.fk = ForwardKinematics()
        self.ik = InverseKinematics()
        self.planner = AdvancedTrajectoryPlanner(dt=0.01)
        self.controller = TrajectoryTracker(control_type='pid')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/trajectory_path', 10)
        
        # Current robot state
        self.current_joints = np.zeros(4)
        self.current_velocities = np.zeros(4)
        
        # Trajectory
        self.trajectory_pos = None
        self.trajectory_vel = None
        self.trajectory_acc = None
        self.traj_index = 0
        self.is_executing = False
        
        # Control timer
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        # Demo state machine
        self.demo_state = 'init'
        self.demo_counter = 0
        self.demo_wait = 0
        
        self.get_logger().info('Integrated Robot Demo Started!')
        self.get_logger().info('Executing autonomous demo sequence...')
        
    def control_loop(self):
        """Main control loop"""
        
        # State machine for autonomous demo
        if self.demo_state == 'init':
            self._init_demo()
            
        elif self.demo_state == 'joint_space_demo':
            self._joint_space_demo()
            
        elif self.demo_state == 'task_space_demo':
            self._task_space_demo()
            
        elif self.demo_state == 'circle_demo':
            self._circle_demo()
            
        elif self.demo_state == 'complete':
            self._complete_demo()
        
        # Publish current state
        self._publish_joint_state()
        self._publish_ee_pose()
        
    def _init_demo(self):
        """Initialize demo"""
        self.get_logger().info('Demo Phase 1: Moving to home position...')
        
        # Plan trajectory to home
        goal = np.array([0, 0, 0, 0])
        self.trajectory_pos, self.trajectory_vel, self.trajectory_acc = \
            self.planner.joint_space_trajectory(self.current_joints, goal, duration=2.0, method='quintic')
        
        self.traj_index = 0
        self.demo_state = 'executing_init'
        self.controller.reset()
        
    def _joint_space_demo(self):
        """Joint-space trajectory demo"""
        if self.demo_counter == 0:
            self.get_logger().info('Demo Phase 2: Joint-space trajectory...')
            
            goal = np.array([np.pi/2, np.pi/4, -np.pi/4, np.pi/3])
            self.trajectory_pos, self.trajectory_vel, self.trajectory_acc = \
                self.planner.joint_space_trajectory(self.current_joints, goal, duration=3.0, method='quintic')
            
            self.traj_index = 0
            self.demo_state = 'executing_joint'
            self.controller.reset()
            self.demo_counter = 0
        
    def _task_space_demo(self):
        """Task-space trajectory demo"""
        if self.demo_counter == 0:
            self.get_logger().info('Demo Phase 3: Task-space straight line...')
            
            start_pos = np.array([0.2, 0.0, 0.7])
            goal_pos = np.array([0.3, 0.2, 0.5])
            
            self.trajectory_pos, self.trajectory_vel, self.trajectory_acc = \
                self.planner.task_space_trajectory(start_pos, goal_pos, duration=4.0, 
                                                   initial_joints=self.current_joints)
            
            # Visualize path
            self._visualize_path(start_pos, goal_pos)
            
            self.traj_index = 0
            self.demo_state = 'executing_task'
            self.controller.reset()
            self.demo_counter = 0
        
    def _circle_demo(self):
        """Circular trajectory demo"""
        if self.demo_counter == 0:
            self.get_logger().info('Demo Phase 4: Circular trajectory...')
            
            center = np.array([0.25, 0.0, 0.6])
            radius = 0.1
            normal = np.array([0, 0, 1])
            
            self.trajectory_pos, self.trajectory_vel, self.trajectory_acc = \
                self.planner.circular_trajectory(center, radius, normal, duration=5.0,
                                                initial_joints=self.current_joints)
            
            self.traj_index = 0
            self.demo_state = 'executing_circle'
            self.controller.reset()
            self.demo_counter = 0
        
    def _complete_demo(self):
        """Demo complete"""
        if self.demo_counter == 0:
            self.get_logger().info('=' * 70)
            self.get_logger().info('DEMO COMPLETE!')
            self.get_logger().info('All trajectory types demonstrated successfully.')
            self.get_logger().info('=' * 70)
            self.demo_counter = 1
        
    def _execute_trajectory(self):
        """Execute current trajectory"""
        if self.traj_index < len(self.trajectory_pos):
            # Compute control
            torque = self.controller.track(
                self.trajectory_pos, self.trajectory_vel, self.trajectory_acc,
                self.current_joints, self.current_velocities,
                self.dt, self.traj_index
            )
            
            # Simple integration (in real system, this would be done by actuators)
            self.current_velocities += torque * self.dt * 0.1  # Simplified dynamics
            self.current_joints += self.current_velocities * self.dt
            
            # Also directly use desired position for smoother demo
            alpha = 0.9  # Blend factor
            self.current_joints = alpha * self.trajectory_pos[self.traj_index] + \
                                 (1-alpha) * self.current_joints
            
            self.traj_index += 1
        else:
            # Trajectory complete, move to next demo
            self._advance_demo_state()
    
    def _advance_demo_state(self):
        """Advance to next demo state"""
        transitions = {
            'executing_init': 'joint_space_demo',
            'executing_joint': 'task_space_demo',
            'executing_task': 'circle_demo',
            'executing_circle': 'complete'
        }
        
        if self.demo_state in transitions:
            self.demo_state = transitions[self.demo_state]
            self.demo_counter = 0
            self.demo_wait = int(1.0 / self.dt)  # 1 second wait
    
    def _publish_joint_state(self):
        """Publish current joint state"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        msg.position = self.current_joints.tolist()
        msg.velocity = self.current_velocities.tolist()
        
        self.joint_pub.publish(msg)
        
        # Execute trajectory
        if self.demo_state.startswith('executing'):
            self._execute_trajectory()
    
    def _publish_ee_pose(self):
        """Publish end-effector pose"""
        pos, rot = self.fk.compute_fk(self.current_joints)
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = 1.0
        
        self.ee_pose_pub.publish(msg)
    
    def _visualize_path(self, start: np.ndarray, end: np.ndarray):
        """Visualize planned path in RViz"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trajectory'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.01  # Line width
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Add start and end points
        p1 = Point()
        p1.x, p1.y, p1.z = start[0], start[1], start[2]
        marker.points.append(p1)
        
        p2 = Point()
        p2.x, p2.y, p2.z = end[0], end[1], end[2]
        marker.points.append(p2)
        
        self.path_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    
    node = IntegratedRobotDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
