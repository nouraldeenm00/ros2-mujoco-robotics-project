"""
Forward Kinematics Calculator for 4-DOF Robot Arm

This module implements forward kinematics using the geometric approach.
Given joint angles, it computes the end-effector position and orientation.

Robot Structure:
- Joint 1: Base rotation (Z-axis)
- Joint 2: Shoulder (Y-axis)
- Joint 3: Elbow (Y-axis)
- Joint 4: Wrist rotation (Z-axis)
"""

import numpy as np
from typing import Tuple, List


class ForwardKinematics:
    """Forward kinematics solver for the 4-DOF robot arm"""
    
    def __init__(self):
        """
        Initialize robot parameters based on URDF model
        All dimensions in meters
        """
        # Link lengths (from URDF)
        self.base_height = 0.05
        self.link1_length = 0.3
        self.link2_length = 0.25
        self.link3_length = 0.2
        self.ee_length = 0.1
        
        # Total height offset from world to link1 base
        self.z_offset = self.base_height
        
    def rotation_matrix_z(self, theta: float) -> np.ndarray:
        """
        Create rotation matrix around Z-axis
        
        Args:
            theta: Rotation angle in radians
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    
    def rotation_matrix_y(self, theta: float) -> np.ndarray:
        """
        Create rotation matrix around Y-axis
        
        Args:
            theta: Rotation angle in radians
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        c = np.cos(theta)
        s = np.sin(theta)
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1]
        ])
    
    def translation_matrix(self, x: float, y: float, z: float) -> np.ndarray:
        """
        Create translation matrix
        
        Args:
            x, y, z: Translation in each axis
            
        Returns:
            4x4 homogeneous transformation matrix
        """
        return np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
    
    def compute_fk(self, joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for given joint angles
        
        Args:
            joint_angles: List of 4 joint angles [theta1, theta2, theta3, theta4] in radians
            
        Returns:
            Tuple of (position, rotation_matrix)
            - position: 3D position [x, y, z]
            - rotation_matrix: 3x3 rotation matrix of end effector
        """
        theta1, theta2, theta3, theta4 = joint_angles
        
        # Transformation from world to base
        T0 = self.translation_matrix(0, 0, self.z_offset)
        
        # Joint 1: Base rotation around Z
        T1 = self.rotation_matrix_z(theta1)
        
        # Move up link1
        T1_link = self.translation_matrix(0, 0, self.link1_length)
        
        # Joint 2: Shoulder rotation around Y
        T2 = self.rotation_matrix_y(theta2)
        
        # Move up link2
        T2_link = self.translation_matrix(0, 0, self.link2_length)
        
        # Joint 3: Elbow rotation around Y
        T3 = self.rotation_matrix_y(theta3)
        
        # Move up link3
        T3_link = self.translation_matrix(0, 0, self.link3_length)
        
        # Joint 4: Wrist rotation around Z
        T4 = self.rotation_matrix_z(theta4)
        
        # Move to end effector tip
        T4_link = self.translation_matrix(0, 0, self.ee_length)
        
        # Chain all transformations
        T_total = T0 @ T1 @ T1_link @ T2 @ T2_link @ T3 @ T3_link @ T4 @ T4_link
        
        # Extract position and orientation
        position = T_total[:3, 3]
        rotation = T_total[:3, :3]
        
        return position, rotation
    
    def compute_jacobian(self, joint_angles: List[float]) -> np.ndarray:
        """
        Compute the Jacobian matrix for velocity kinematics
        
        Args:
            joint_angles: List of 4 joint angles in radians
            
        Returns:
            6x4 Jacobian matrix [linear_velocity; angular_velocity]
        """
        # Numerical jacobian using finite differences
        epsilon = 1e-6
        jacobian = np.zeros((6, 4))
        
        # Get current end-effector pose
        pos0, rot0 = self.compute_fk(joint_angles)
        
        for i in range(4):
            # Perturb joint i
            perturbed_angles = joint_angles.copy()
            perturbed_angles[i] += epsilon
            pos_perturbed, rot_perturbed = self.compute_fk(perturbed_angles)
            
            # Linear velocity component
            jacobian[:3, i] = (pos_perturbed - pos0) / epsilon
            
            # Angular velocity component (simplified)
            # For a more accurate calculation, use rotation matrix derivatives
            jacobian[3:, i] = 0  # Placeholder for angular velocity
        
        return jacobian


def test_forward_kinematics():
    """Test the forward kinematics with known configurations"""
    fk = ForwardKinematics()
    
    # Test 1: All joints at zero (home position)
    print("Test 1: Home position (all joints at 0)")
    joint_angles = [0, 0, 0, 0]
    pos, rot = fk.compute_fk(joint_angles)
    print(f"Position: {pos}")
    print(f"Expected: approximately [0, 0, {fk.z_offset + fk.link1_length + fk.link2_length + fk.link3_length + fk.ee_length}]")
    print()
    
    # Test 2: 90 degree base rotation
    print("Test 2: 90° base rotation")
    joint_angles = [np.pi/2, 0, 0, 0]
    pos, rot = fk.compute_fk(joint_angles)
    print(f"Position: {pos}")
    print()
    
    # Test 3: 90 degree shoulder bend
    print("Test 3: 90° shoulder bend")
    joint_angles = [0, np.pi/2, 0, 0]
    pos, rot = fk.compute_fk(joint_angles)
    print(f"Position: {pos}")
    print()


if __name__ == "__main__":
    test_forward_kinematics()
