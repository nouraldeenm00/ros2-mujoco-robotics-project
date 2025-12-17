"""
Velocity Kinematics for 4-DOF Robot Arm

Implements forward and inverse velocity kinematics using the Jacobian matrix.
"""

import numpy as np
from typing import Tuple, Optional
from .forward_kinematics import ForwardKinematics


class VelocityKinematics:
    """Velocity kinematics solver using Jacobian"""
    
    def __init__(self):
        """Initialize velocity kinematics solver"""
        self.fk = ForwardKinematics()
        
    def compute_jacobian_numerical(self, joint_angles: np.ndarray, 
                                   epsilon: float = 1e-6) -> np.ndarray:
        """
        Compute Jacobian matrix numerically using finite differences
        
        Args:
            joint_angles: Current joint angles [4]
            epsilon: Perturbation for numerical derivative
            
        Returns:
            6x4 Jacobian matrix [linear_velocity; angular_velocity]
        """
        jacobian = np.zeros((6, 4))
        
        # Get current end-effector pose
        pos0, rot0 = self.fk.compute_fk(joint_angles)
        
        for i in range(4):
            # Perturb joint i
            perturbed_angles = joint_angles.copy()
            perturbed_angles[i] += epsilon
            pos_perturbed, rot_perturbed = self.fk.compute_fk(perturbed_angles)
            
            # Linear velocity (dx/dθ, dy/dθ, dz/dθ)
            jacobian[:3, i] = (pos_perturbed - pos0) / epsilon
            
            # Angular velocity (approximation)
            # For more accuracy, should use rotation matrix difference
            jacobian[3:, i] = 0  # Simplified - can be improved
        
        return jacobian
    
    def compute_jacobian_analytical(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Compute Jacobian matrix analytically
        
        The Jacobian relates joint velocities to end-effector velocities:
        [v_x]       [J₁₁ J₁₂ J₁₃ J₁₄]   [θ̇₁]
        [v_y]       [J₂₁ J₂₂ J₂₃ J₂₄]   [θ̇₂]
        [v_z]   =   [J₃₁ J₃₂ J₃₃ J₃₄] × [θ̇₃]
        [ω_x]       [J₄₁ J₄₂ J₄₃ J₄₄]   [θ̇₄]
        [ω_y]       [J₅₁ J₅₂ J₅₃ J₅₄]
        [ω_z]       [J₆₁ J₆₂ J₆₃ J₆₄]
        
        Args:
            joint_angles: Current joint angles [θ₁, θ₂, θ₃, θ₄]
            
        Returns:
            6x4 Jacobian matrix
        """
        θ1, θ2, θ3, θ4 = joint_angles
        
        # Link lengths
        l1 = self.fk.link1_length
        l2 = self.fk.link2_length
        l3 = self.fk.link3_length
        l4 = self.fk.ee_length
        d0 = self.fk.z_offset
        
        # Sines and cosines
        c1, s1 = np.cos(θ1), np.sin(θ1)
        c2, s2 = np.cos(θ2), np.sin(θ2)
        c3, s3 = np.cos(θ3), np.sin(θ3)
        c23 = np.cos(θ2 + θ3)
        s23 = np.sin(θ2 + θ3)
        
        jacobian = np.zeros((6, 4))
        
        # Linear velocity part (top 3 rows)
        # Column 1 (effect of θ̇₁)
        jacobian[0, 0] = -s1 * (l2*s2 + l3*s23 + l4*s23)
        jacobian[1, 0] = c1 * (l2*s2 + l3*s23 + l4*s23)
        jacobian[2, 0] = 0
        
        # Column 2 (effect of θ̇₂)
        jacobian[0, 1] = c1 * (l2*c2 + l3*c23 + l4*c23)
        jacobian[1, 1] = s1 * (l2*c2 + l3*c23 + l4*c23)
        jacobian[2, 1] = -(l2*s2 + l3*s23 + l4*s23)
        
        # Column 3 (effect of θ̇₃)
        jacobian[0, 2] = c1 * (l3*c23 + l4*c23)
        jacobian[1, 2] = s1 * (l3*c23 + l4*c23)
        jacobian[2, 2] = -(l3*s23 + l4*s23)
        
        # Column 4 (effect of θ̇₄) - wrist rotation doesn't affect position
        jacobian[0, 3] = 0
        jacobian[1, 3] = 0
        jacobian[2, 3] = 0
        
        # Angular velocity part (bottom 3 rows)
        # Joint 1 rotates around Z
        jacobian[3, 0] = 0
        jacobian[4, 0] = 0
        jacobian[5, 0] = 1
        
        # Joint 2 rotates around Y (in rotated frame)
        jacobian[3, 1] = -s1
        jacobian[4, 1] = c1
        jacobian[5, 1] = 0
        
        # Joint 3 rotates around Y (in rotated frame)
        jacobian[3, 2] = -s1
        jacobian[4, 2] = c1
        jacobian[5, 2] = 0
        
        # Joint 4 rotates around Z (in rotated frame)
        jacobian[3, 3] = -s1*s23
        jacobian[4, 3] = c1*s23
        jacobian[5, 3] = c23
        
        return jacobian
    
    def forward_velocity(self, joint_angles: np.ndarray, 
                        joint_velocities: np.ndarray,
                        use_analytical: bool = True) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute end-effector velocity from joint velocities
        
        V_ee = J(q) × q̇
        
        Args:
            joint_angles: Current joint angles [4]
            joint_velocities: Joint velocities [4]
            use_analytical: Use analytical Jacobian (True) or numerical (False)
            
        Returns:
            Tuple of (linear_velocity [3], angular_velocity [3])
        """
        if use_analytical:
            J = self.compute_jacobian_analytical(joint_angles)
        else:
            J = self.compute_jacobian_numerical(joint_angles)
        
        # Compute end-effector velocity
        ee_velocity = J @ joint_velocities
        
        linear_vel = ee_velocity[:3]
        angular_vel = ee_velocity[3:]
        
        return linear_vel, angular_vel
    
    def inverse_velocity(self, joint_angles: np.ndarray,
                        linear_velocity: np.ndarray,
                        angular_velocity: Optional[np.ndarray] = None,
                        use_analytical: bool = True) -> Optional[np.ndarray]:
        """
        Compute joint velocities from desired end-effector velocity
        
        q̇ = J⁺(q) × V_ee
        
        Where J⁺ is the pseudo-inverse of the Jacobian
        
        Args:
            joint_angles: Current joint angles [4]
            linear_velocity: Desired linear velocity [3]
            angular_velocity: Desired angular velocity [3] (optional)
            use_analytical: Use analytical Jacobian (True) or numerical (False)
            
        Returns:
            Joint velocities [4] or None if singular
        """
        if use_analytical:
            J = self.compute_jacobian_analytical(joint_angles)
        else:
            J = self.compute_jacobian_numerical(joint_angles)
        
        # Construct desired velocity vector
        if angular_velocity is None:
            angular_velocity = np.zeros(3)
        
        desired_velocity = np.concatenate([linear_velocity, angular_velocity])
        
        # Compute pseudo-inverse
        try:
            J_pinv = np.linalg.pinv(J)
            joint_velocities = J_pinv @ desired_velocity
            return joint_velocities
        except np.linalg.LinAlgError:
            # Singular configuration
            return None
    
    def check_singularity(self, joint_angles: np.ndarray, 
                         threshold: float = 1e-3) -> bool:
        """
        Check if configuration is near a singularity
        
        Args:
            joint_angles: Joint angles to check
            threshold: Singularity threshold (determinant)
            
        Returns:
            True if near singularity, False otherwise
        """
        J = self.compute_jacobian_analytical(joint_angles)
        
        # For rectangular matrix, check smallest singular value
        singular_values = np.linalg.svd(J, compute_uv=False)
        min_sv = np.min(singular_values)
        
        return min_sv < threshold
    
    def manipulability(self, joint_angles: np.ndarray) -> float:
        """
        Compute manipulability measure (Yoshikawa)
        
        w(q) = √det(J × Jᵀ)
        
        Higher values indicate better manipulability
        
        Args:
            joint_angles: Current joint angles
            
        Returns:
            Manipulability index
        """
        J = self.compute_jacobian_analytical(joint_angles)
        
        # Use only linear velocity part (first 3 rows)
        J_linear = J[:3, :]
        
        manipulability = np.sqrt(np.linalg.det(J_linear @ J_linear.T))
        
        return manipulability


def test_velocity_kinematics():
    """Test velocity kinematics"""
    vk = VelocityKinematics()
    
    print("=" * 60)
    print("Velocity Kinematics Testing")
    print("=" * 60)
    print()
    
    # Test 1: Jacobian computation
    print("Test 1: Jacobian Computation")
    print("-" * 60)
    joint_angles = np.array([0, np.pi/4, 0, 0])
    
    J_analytical = vk.compute_jacobian_analytical(joint_angles)
    J_numerical = vk.compute_jacobian_numerical(joint_angles)
    
    print(f"Configuration: {joint_angles}")
    print(f"\nAnalytical Jacobian:\n{J_analytical}")
    print(f"\nNumerical Jacobian:\n{J_numerical}")
    print(f"\nDifference (should be small):\n{np.abs(J_analytical - J_numerical).max():.6f}")
    print()
    
    # Test 2: Forward velocity
    print("Test 2: Forward Velocity Kinematics")
    print("-" * 60)
    joint_velocities = np.array([0.1, 0.2, -0.1, 0.0])
    
    linear_vel, angular_vel = vk.forward_velocity(joint_angles, joint_velocities)
    
    print(f"Joint velocities: {joint_velocities}")
    print(f"End-effector linear velocity: {linear_vel}")
    print(f"End-effector angular velocity: {angular_vel}")
    print()
    
    # Test 3: Inverse velocity
    print("Test 3: Inverse Velocity Kinematics")
    print("-" * 60)
    desired_linear_vel = np.array([0.1, 0.0, -0.05])
    
    computed_joint_vel = vk.inverse_velocity(joint_angles, desired_linear_vel)
    
    if computed_joint_vel is not None:
        print(f"Desired linear velocity: {desired_linear_vel}")
        print(f"Computed joint velocities: {computed_joint_vel}")
        
        # Verify
        linear_vel_check, _ = vk.forward_velocity(joint_angles, computed_joint_vel)
        print(f"Achieved linear velocity: {linear_vel_check}")
        print(f"Error: {np.linalg.norm(linear_vel_check - desired_linear_vel):.6f}")
    else:
        print("Configuration is singular!")
    print()
    
    # Test 4: Singularity check
    print("Test 4: Singularity Detection")
    print("-" * 60)
    test_configs = [
        np.array([0, 0, 0, 0]),
        np.array([0, np.pi/2, -np.pi/2, 0]),  # Fully extended
        np.array([0, np.pi/4, np.pi/4, 0]),
    ]
    
    for config in test_configs:
        is_singular = vk.check_singularity(config)
        manip = vk.manipulability(config)
        print(f"Config {config}: Singular={is_singular}, Manipulability={manip:.4f}")
    print()
    
    print("=" * 60)


if __name__ == "__main__":
    test_velocity_kinematics()
