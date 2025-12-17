"""
Acceleration Kinematics for 4-DOF Robot Arm

Implements forward and inverse acceleration kinematics.
"""

import numpy as np
from typing import Tuple, Optional
from .forward_kinematics import ForwardKinematics
from .velocity_kinematics import VelocityKinematics


class AccelerationKinematics:
    """Acceleration kinematics solver"""
    
    def __init__(self):
        """Initialize acceleration kinematics solver"""
        self.fk = ForwardKinematics()
        self.vk = VelocityKinematics()
        
    def compute_jacobian_derivative(self, joint_angles: np.ndarray,
                                    joint_velocities: np.ndarray,
                                    epsilon: float = 1e-6) -> np.ndarray:
        """
        Compute time derivative of Jacobian numerically
        
        J̇ = dJ/dt
        
        Args:
            joint_angles: Current joint angles [4]
            joint_velocities: Current joint velocities [4]
            epsilon: Time step for numerical derivative
            
        Returns:
            6x4 Jacobian derivative matrix
        """
        # Get current Jacobian
        J0 = self.vk.compute_jacobian_analytical(joint_angles)
        
        # Perturb forward in time
        angles_perturbed = joint_angles + epsilon * joint_velocities
        J1 = self.vk.compute_jacobian_analytical(angles_perturbed)
        
        # Numerical derivative
        J_dot = (J1 - J0) / epsilon
        
        return J_dot
    
    def forward_acceleration(self, joint_angles: np.ndarray,
                            joint_velocities: np.ndarray,
                            joint_accelerations: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute end-effector acceleration from joint accelerations
        
        A_ee = J(q) × q̈ + J̇(q, q̇) × q̇
        
        Args:
            joint_angles: Current joint angles [4]
            joint_velocities: Current joint velocities [4]
            joint_accelerations: Joint accelerations [4]
            
        Returns:
            Tuple of (linear_acceleration [3], angular_acceleration [3])
        """
        # Get Jacobian and its derivative
        J = self.vk.compute_jacobian_analytical(joint_angles)
        J_dot = self.compute_jacobian_derivative(joint_angles, joint_velocities)
        
        # Compute end-effector acceleration
        # A = J × q̈ + J̇ × q̇
        ee_acceleration = J @ joint_accelerations + J_dot @ joint_velocities
        
        linear_acc = ee_acceleration[:3]
        angular_acc = ee_acceleration[3:]
        
        return linear_acc, angular_acc
    
    def inverse_acceleration(self, joint_angles: np.ndarray,
                            joint_velocities: np.ndarray,
                            linear_acceleration: np.ndarray,
                            angular_acceleration: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Compute joint accelerations from desired end-effector acceleration
        
        q̈ = J⁺(q) × [A_ee - J̇(q, q̇) × q̇]
        
        Args:
            joint_angles: Current joint angles [4]
            joint_velocities: Current joint velocities [4]
            linear_acceleration: Desired linear acceleration [3]
            angular_acceleration: Desired angular acceleration [3] (optional)
            
        Returns:
            Joint accelerations [4] or None if singular
        """
        # Get Jacobian and its derivative
        J = self.vk.compute_jacobian_analytical(joint_angles)
        J_dot = self.compute_jacobian_derivative(joint_angles, joint_velocities)
        
        # Construct desired acceleration vector
        if angular_acceleration is None:
            angular_acceleration = np.zeros(3)
        
        desired_acceleration = np.concatenate([linear_acceleration, angular_acceleration])
        
        # Compute bias term
        bias = J_dot @ joint_velocities
        
        # Solve for joint accelerations
        try:
            J_pinv = np.linalg.pinv(J)
            joint_accelerations = J_pinv @ (desired_acceleration - bias)
            return joint_accelerations
        except np.linalg.LinAlgError:
            # Singular configuration
            return None
    
    def compute_coriolis_centrifugal(self, joint_angles: np.ndarray,
                                     joint_velocities: np.ndarray) -> np.ndarray:
        """
        Compute Coriolis and centrifugal acceleration terms
        
        This is the J̇ × q̇ term in the acceleration equation
        
        Args:
            joint_angles: Current joint angles
            joint_velocities: Current joint velocities
            
        Returns:
            Coriolis/centrifugal acceleration [6]
        """
        J_dot = self.compute_jacobian_derivative(joint_angles, joint_velocities)
        return J_dot @ joint_velocities


def test_acceleration_kinematics():
    """Test acceleration kinematics"""
    ak = AccelerationKinematics()
    
    print("=" * 60)
    print("Acceleration Kinematics Testing")
    print("=" * 60)
    print()
    
    # Test configuration
    joint_angles = np.array([0, np.pi/6, np.pi/6, 0])
    joint_velocities = np.array([0.1, 0.2, -0.1, 0.05])
    joint_accelerations = np.array([0.05, -0.1, 0.15, 0.0])
    
    # Test 1: Jacobian derivative
    print("Test 1: Jacobian Derivative")
    print("-" * 60)
    J_dot = ak.compute_jacobian_derivative(joint_angles, joint_velocities)
    print(f"Jacobian derivative shape: {J_dot.shape}")
    print(f"J̇ matrix:\n{J_dot}")
    print()
    
    # Test 2: Forward acceleration
    print("Test 2: Forward Acceleration Kinematics")
    print("-" * 60)
    linear_acc, angular_acc = ak.forward_acceleration(
        joint_angles, joint_velocities, joint_accelerations
    )
    
    print(f"Joint angles: {joint_angles}")
    print(f"Joint velocities: {joint_velocities}")
    print(f"Joint accelerations: {joint_accelerations}")
    print(f"\nEnd-effector linear acceleration: {linear_acc}")
    print(f"End-effector angular acceleration: {angular_acc}")
    print()
    
    # Test 3: Inverse acceleration
    print("Test 3: Inverse Acceleration Kinematics")
    print("-" * 60)
    desired_linear_acc = np.array([0.05, 0.0, -0.1])
    
    computed_joint_acc = ak.inverse_acceleration(
        joint_angles, joint_velocities, desired_linear_acc
    )
    
    if computed_joint_acc is not None:
        print(f"Desired linear acceleration: {desired_linear_acc}")
        print(f"Computed joint accelerations: {computed_joint_acc}")
        
        # Verify
        linear_acc_check, _ = ak.forward_acceleration(
            joint_angles, joint_velocities, computed_joint_acc
        )
        print(f"Achieved linear acceleration: {linear_acc_check}")
        print(f"Error: {np.linalg.norm(linear_acc_check - desired_linear_acc):.6f}")
    else:
        print("Configuration is singular!")
    print()
    
    # Test 4: Coriolis/centrifugal terms
    print("Test 4: Coriolis and Centrifugal Terms")
    print("-" * 60)
    coriolis = ak.compute_coriolis_centrifugal(joint_angles, joint_velocities)
    print(f"Coriolis/centrifugal acceleration:\n{coriolis}")
    print()
    
    print("=" * 60)


if __name__ == "__main__":
    test_acceleration_kinematics()
