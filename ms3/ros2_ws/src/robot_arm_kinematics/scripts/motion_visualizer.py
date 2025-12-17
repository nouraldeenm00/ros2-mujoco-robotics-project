#!/usr/bin/env python3
"""
Real-time RViz2 Motion Visualizer
Shows robot picking and placing object with proper gripper control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
import numpy as np
import time

class RobotMotionPublisher(Node):
    def __init__(self):
        super().__init__('robot_motion_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.gripper_pub = self.create_publisher(Bool, 'gripper_state', 10)
        self.pick_pos_pub = self.create_publisher(Point, 'pick_location', 10)
        self.place_pos_pub = self.create_publisher(Point, 'place_location', 10)
        
        # Import kinematics
        from robot_arm_kinematics.forward_kinematics import ForwardKinematics
        from robot_arm_kinematics.inverse_kinematics import InverseKinematics
        from robot_arm_kinematics.advanced_trajectory import AdvancedTrajectoryPlanner
        
        self.fk = ForwardKinematics()
        self.ik = InverseKinematics()
        self.traj = AdvancedTrajectoryPlanner()
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
    def publish_gripper_state(self, closed):
        """Publish gripper state"""
        msg = Bool()
        msg.data = closed
        self.gripper_pub.publish(msg)
        
    def publish_joint_state(self, positions, velocities=None):
        """Publish joint positions to RViz2"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions.tolist()
        msg.velocity = (velocities if velocities is not None else np.zeros(4)).tolist()
        self.publisher_.publish(msg)
        
    def smooth_move(self, start_joints, end_joints, duration=3.0, description=""):
        """Execute smooth trajectory from start to end"""
        if description:
            self.get_logger().info(f"  {description}")
        
        positions, velocities, _ = self.traj.joint_space_trajectory(
            start_joints, end_joints, duration=duration
        )
        
        # Publish at 50Hz
        dt = duration / len(positions)
        for i, config in enumerate(positions):
            self.publish_joint_state(config, velocities[i])
            time.sleep(dt)
        
        return end_joints
    
    def pick_and_place_demo(self):
        """Complete pick and place demonstration"""
        self.get_logger().info("="*60)
        self.get_logger().info("PICK AND PLACE DEMONSTRATION")
        self.get_logger().info("="*60)
        
        # Initial positions
        home = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Random position generation
        RADIUS = 0.12  # 12cm radius (smaller to stay within workspace)
        MIN_DISTANCE = 0.15  # Minimum 15cm distance between pick and place
        reference_x = 0.35
        reference_y = 0.0
        reference_z = 0.65
        
        # Generate random pick and place positions with constraints and reachability checks
        valid_positions = False
        attempts = 0
        while not valid_positions:
            attempts += 1
            # Generate random pick position within radius
            angle1 = np.random.uniform(0, 2*np.pi)
            distance1 = np.random.uniform(0, RADIUS)
            pick_x = reference_x + distance1 * np.cos(angle1)
            pick_y = reference_y + distance1 * np.sin(angle1)
            pick_pos = np.array([pick_x, pick_y, reference_z])

            # Generate random place position within radius
            angle2 = np.random.uniform(0, 2*np.pi)
            distance2 = np.random.uniform(0, RADIUS)
            place_x = reference_x + distance2 * np.cos(angle2)
            place_y = reference_y + distance2 * np.sin(angle2)
            place_pos = np.array([place_x, place_y, reference_z])

            # Minimum planar separation
            separation = np.linalg.norm(pick_pos[:2] - place_pos[:2])
            if separation < MIN_DISTANCE:
                continue

            # Check IK reachability for approach points (10cm above)
            above_pick = pick_pos.copy(); above_pick[2] += 0.10
            above_place = place_pos.copy(); above_place[2] += 0.10
            ik_pick_above = self.ik.compute_ik(above_pick)
            ik_place_above = self.ik.compute_ik(above_place)
            ik_pick = self.ik.compute_ik(pick_pos)
            ik_place = self.ik.compute_ik(place_pos)

            if (ik_pick_above is not None) and (ik_place_above is not None) and (ik_pick is not None) and (ik_place is not None):
                valid_positions = True
                self.get_logger().info(f"    Pick location: {pick_pos.tolist()}")
                self.get_logger().info(f"    Place location: {place_pos.tolist()}")
                self.get_logger().info(f"    Distance between flags: {separation:.3f} m (attempt {attempts})")
        
        # Publish positions for scene markers to use
        pick_msg = Point()
        pick_msg.x, pick_msg.y, pick_msg.z = float(pick_pos[0]), float(pick_pos[1]), float(pick_pos[2])
        self.pick_pos_pub.publish(pick_msg)
        time.sleep(0.1)
        
        place_msg = Point()
        place_msg.x, place_msg.y, place_msg.z = float(place_pos[0]), float(place_pos[1]), float(place_pos[2])
        self.place_pos_pub.publish(place_msg)
        time.sleep(0.1)
        
        # Open gripper
        self.publish_gripper_state(False)
        
        # Step 1: Start at home
        self.get_logger().info("\n1️⃣  Starting at HOME position")
        # Locations already logged above
        self.publish_joint_state(home)
        time.sleep(2.0)
        
        # Step 2: Move above pick location
        self.get_logger().info("\n2️⃣  Moving above PICK location (green flag)")
        above_pick = pick_pos.copy()
        above_pick[2] += 0.10  # 10cm above
        pick_joints_above = self.ik.compute_ik(above_pick)
        
        if pick_joints_above is not None:
            current_joints = self.smooth_move(
                home, np.array(pick_joints_above), 
                duration=4.0, description="Moving to above pick location..."
            )
            time.sleep(0.5)
        else:
            self.get_logger().error("Cannot reach above pick location!")
            return
        
        # Step 3: Lower to pick location
        self.get_logger().info("\n3️⃣  Lowering to PICK location")
        pick_joints = self.ik.compute_ik(pick_pos)
        
        if pick_joints is not None:
            current_joints = self.smooth_move(
                current_joints, np.array(pick_joints),
                duration=2.0, description="Descending..."
            )
            time.sleep(0.5)
        else:
            self.get_logger().error("Cannot reach pick location!")
            return
        
        # Step 4: Close gripper (GRASP)
        self.get_logger().info("\n4️⃣  GRASPING object")
        self.publish_gripper_state(True)
        time.sleep(1.0)
        
        # Step 5: Lift object
        self.get_logger().info("\n5️⃣  Lifting object")
        current_joints = self.smooth_move(
            current_joints, np.array(pick_joints_above),
            duration=2.0, description="Lifting..."
        )
        time.sleep(0.5)
        
        # Step 6: Move above place location
        self.get_logger().info("\n6️⃣  Moving to PLACE location (red flag)")
        above_place = place_pos.copy()
        above_place[2] += 0.10  # 10cm above
        place_joints_above = self.ik.compute_ik(above_place)
        
        if place_joints_above is not None:
            current_joints = self.smooth_move(
                current_joints, np.array(place_joints_above),
                duration=4.0, description="Moving with object..."
            )
            time.sleep(0.5)
        else:
            self.get_logger().error("Cannot reach above place location!")
            return
        
        # Step 7: Lower to place location
        self.get_logger().info("\n7️⃣  Lowering to PLACE location")
        place_joints = self.ik.compute_ik(place_pos)
        
        if place_joints is not None:
            current_joints = self.smooth_move(
                current_joints, np.array(place_joints),
                duration=2.0, description="Descending..."
            )
            time.sleep(0.5)
        else:
            self.get_logger().error("Cannot reach place location!")
            return
        
        # Step 8: Open gripper (RELEASE)
        self.get_logger().info("\n8️⃣  RELEASING object")
        self.get_logger().info(f"    Object position at release: {place_pos.tolist()}")
        self.publish_gripper_state(False)
        time.sleep(1.0)
        
        # Step 9: Lift up
        self.get_logger().info("\n9️⃣  Lifting up")
        current_joints = self.smooth_move(
            current_joints, np.array(place_joints_above),
            duration=2.0, description="Moving up..."
        )
        time.sleep(0.5)
        
        # Step 10: Return home
        self.get_logger().info("\n🔟  Returning HOME")
        self.smooth_move(
            current_joints, home,
            duration=4.0, description="Returning to home position..."
        )
        time.sleep(1.0)
        
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("✅ PICK AND PLACE COMPLETE!")
        self.get_logger().info("="*60)


def main(args=None):
    rclpy.init(args=args)
    publisher = RobotMotionPublisher()
    
    try:
        publisher.pick_and_place_demo()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        publisher.get_logger().error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
