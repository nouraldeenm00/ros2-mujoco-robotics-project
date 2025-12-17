#!/usr/bin/env python3
"""
Scene Marker Publisher
Publishes visual markers for object, start flag, and end flag in RViz2
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import numpy as np


class SceneMarkerPublisher(Node):
    def __init__(self):
        super().__init__('scene_marker_publisher')
        self.publisher_ = self.create_publisher(MarkerArray, 'scene_markers', 10)
        self.timer = self.create_timer(0.05, self.publish_markers)
        
        # Subscribe to joint states to track end effector
        self.joint_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_callback, 10)
        
        # Subscribe to gripper state
        self.gripper_sub = self.create_subscription(
            Bool, 'gripper_state', self.gripper_callback, 10)
        
        # Subscribe to dynamic pick and place locations
        self.pick_pos_sub = self.create_subscription(
            Point, 'pick_location', self.pick_pos_callback, 10)
        self.place_pos_sub = self.create_subscription(
            Point, 'place_location', self.place_pos_callback, 10)
        
        # Object/flag geometry
        self.object_size = 0.08  # cube side (m)
        self.object_half = self.object_size / 2.0
        self.clearance = 0.005   # 5mm visual clearance above flag plane

        # Positions
        self.start_pos = [0.35, 0.1, 0.65]   # Start flag = pick location
        self.end_pos = [0.3, -0.15, 0.65]   # End flag = place location
        # Object center initially at pick location (no artificial offset)
        self.object_pos = [self.start_pos[0], self.start_pos[1], self.start_pos[2]]
        # Attachment offset (object relative to EE) computed on grasp
        self.attach_offset = np.array([0.0, 0.0, -0.05])
        
        # State
        self.gripper_closed = False
        self.end_effector_pos = [0.0, 0.0, 0.9]
        
        # FK for computing end effector position
        from robot_arm_kinematics.forward_kinematics import ForwardKinematics
        self.fk = ForwardKinematics()
        
    def joint_callback(self, msg):
        """Update end effector position from joint states"""
        if len(msg.position) >= 4:
            joints = np.array(msg.position[:4])
            pos, _ = self.fk.compute_fk(joints)
            self.end_effector_pos = pos.flatten().tolist()
            
            # If gripper is closed, object follows end effector with locked offset
            if self.gripper_closed:
                ee = np.array(self.end_effector_pos)
                obj = ee + self.attach_offset
                self.object_pos = obj.tolist()
    
    def gripper_callback(self, msg):
        """Update gripper state"""
        was_closed = self.gripper_closed
        self.gripper_closed = msg.data
        
        # If gripper just closed (picking up), lock current relative offset (prevents jump)
        if not was_closed and self.gripper_closed:
            ee = np.array(self.end_effector_pos)
            obj = np.array(self.object_pos)
            self.attach_offset = obj - ee
        # If gripper just released, do not snap; leave object where it is
        elif was_closed and not self.gripper_closed:
            pass
    
    def pick_pos_callback(self, msg):
        """Update pick location from motion_visualizer"""
        self.start_pos = [msg.x, msg.y, msg.z]
        # Initialize object at pick flag center when gripper is open (continuous)
        if not self.gripper_closed:
            self.object_pos = [self.start_pos[0], self.start_pos[1], self.start_pos[2]]
    
    def place_pos_callback(self, msg):
        """Update place location from motion_visualizer"""
        self.end_pos = [msg.x, msg.y, msg.z]
        
    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Object (Yellow Box)
        object_marker = Marker()
        object_marker.header.frame_id = "world"
        object_marker.header.stamp = self.get_clock().now().to_msg()
        object_marker.ns = "scene"
        object_marker.id = 0
        object_marker.type = Marker.CUBE
        object_marker.action = Marker.ADD
        object_marker.pose.position.x = self.object_pos[0]
        object_marker.pose.position.y = self.object_pos[1]
        object_marker.pose.position.z = self.object_pos[2]
        object_marker.pose.orientation.w = 1.0
        object_marker.scale.x = 0.08
        object_marker.scale.y = 0.08
        object_marker.scale.z = 0.08
        object_marker.color.r = 1.0
        object_marker.color.g = 1.0
        object_marker.color.b = 0.0
        object_marker.color.a = 0.9
        marker_array.markers.append(object_marker)
        
        # Start Flag (Green)
        start_pole = Marker()
        start_pole.header.frame_id = "world"
        start_pole.header.stamp = self.get_clock().now().to_msg()
        start_pole.ns = "scene"
        start_pole.id = 1
        start_pole.type = Marker.CYLINDER
        start_pole.action = Marker.ADD
        start_pole.pose.position.x = self.start_pos[0]
        start_pole.pose.position.y = self.start_pos[1]
        start_pole.pose.position.z = 0.325
        start_pole.pose.orientation.w = 1.0
        start_pole.scale.x = 0.01
        start_pole.scale.y = 0.01
        start_pole.scale.z = 0.65
        start_pole.color.r = 0.0
        start_pole.color.g = 0.8
        start_pole.color.b = 0.0
        start_pole.color.a = 1.0
        marker_array.markers.append(start_pole)
        
        start_flag = Marker()
        start_flag.header.frame_id = "world"
        start_flag.header.stamp = self.get_clock().now().to_msg()
        start_flag.ns = "scene"
        start_flag.id = 2
        start_flag.type = Marker.CUBE
        start_flag.action = Marker.ADD
        start_flag.pose.position.x = self.start_pos[0] + 0.04
        start_flag.pose.position.y = self.start_pos[1]
        # Visual slab positioned just below cube bottom for contact appearance
        start_flag.pose.position.z = self.start_pos[2] - (self.object_half + self.clearance - 0.005)
        start_flag.pose.orientation.w = 1.0
        start_flag.scale.x = 0.08
        start_flag.scale.y = 0.06
        start_flag.scale.z = 0.01
        start_flag.color.r = 0.0
        start_flag.color.g = 1.0
        start_flag.color.b = 0.0
        start_flag.color.a = 1.0
        marker_array.markers.append(start_flag)
        
        # End Flag (Red)
        end_pole = Marker()
        end_pole.header.frame_id = "world"
        end_pole.header.stamp = self.get_clock().now().to_msg()
        end_pole.ns = "scene"
        end_pole.id = 3
        end_pole.type = Marker.CYLINDER
        end_pole.action = Marker.ADD
        end_pole.pose.position.x = self.end_pos[0]
        end_pole.pose.position.y = self.end_pos[1]
        end_pole.pose.position.z = 0.325
        end_pole.pose.orientation.w = 1.0
        end_pole.scale.x = 0.01
        end_pole.scale.y = 0.01
        end_pole.scale.z = 0.65
        end_pole.color.r = 0.8
        end_pole.color.g = 0.0
        end_pole.color.b = 0.0
        end_pole.color.a = 1.0
        marker_array.markers.append(end_pole)
        
        end_flag = Marker()
        end_flag.header.frame_id = "world"
        end_flag.header.stamp = self.get_clock().now().to_msg()
        end_flag.ns = "scene"
        end_flag.id = 4
        end_flag.type = Marker.CUBE
        end_flag.action = Marker.ADD
        end_flag.pose.position.x = self.end_pos[0] + 0.04
        end_flag.pose.position.y = self.end_pos[1]
        end_flag.pose.position.z = self.end_pos[2] - (self.object_half + self.clearance - 0.005)
        end_flag.pose.orientation.w = 1.0
        end_flag.scale.x = 0.08
        end_flag.scale.y = 0.06
        end_flag.scale.z = 0.01
        end_flag.color.r = 1.0
        end_flag.color.g = 0.0
        end_flag.color.b = 0.0
        end_flag.color.a = 1.0
        marker_array.markers.append(end_flag)
        
        # Ground grid
        grid = Marker()
        grid.header.frame_id = "world"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.ns = "scene"
        grid.id = 5
        grid.type = Marker.CUBE
        grid.action = Marker.ADD
        grid.pose.position.x = 0.0
        grid.pose.position.y = 0.0
        grid.pose.position.z = -0.01
        grid.pose.orientation.w = 1.0
        grid.scale.x = 2.0
        grid.scale.y = 2.0
        grid.scale.z = 0.01
        grid.color.r = 0.3
        grid.color.g = 0.3
        grid.color.b = 0.3
        grid.color.a = 0.3
        marker_array.markers.append(grid)
        
        self.publisher_.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SceneMarkerPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        # Avoid noisy shutdown errors
        node.get_logger().error(f"Scene markers exception: {e}")
    finally:
        try:
            node.destroy_node()
        finally:
            # Only shutdown if context is still valid
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
