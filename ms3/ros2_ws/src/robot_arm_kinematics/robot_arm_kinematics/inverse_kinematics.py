"""
Inverse Kinematics Solver for 4-DOF Robot Arm

This module implements inverse kinematics using geometric and analytical methods.
Given a desired end-effector position and orientation, it computes the required joint angles.

For a 4-DOF arm:
- We can control 3D position (x, y, z)
- And one rotational DOF (typically around Z)
"""

import numpy as np
from typing import Optional, List, Tuple
from .forward_kinematics import ForwardKinematics


class InverseKinematics:
    """Inverse kinematics solver for the 4-DOF robot arm"""
    
    def __init__(self):
        """Initialize IK solver with robot parameters"""
        self.fk = ForwardKinematics()
        
        # Joint limits (from URDF)
        self.joint_limits = [
            (-np.pi, np.pi),      # Joint 1: base rotation
            (-np.pi/2, np.pi/2),  # Joint 2: shoulder
            (-np.pi/2, np.pi/2),  # Joint 3: elbow
            (-np.pi, np.pi)       # Joint 4: wrist
        ]
        
    def compute_ik(self, target_pos: np.ndarray, wrist_angle: float = 0.0, 
                   initial_guess: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        Compute inverse kinematics for target position
        
        Args:
            target_pos: Desired end-effector position [x, y, z]
            wrist_angle: Desired wrist rotation angle (joint 4)
            initial_guess: Initial joint angles for iterative solver
            
        Returns:
            List of 4 joint angles if solution exists, None otherwise
        """
        # Method 1: Geometric approach (analytical solution)
        solution = self._geometric_ik(target_pos, wrist_angle)
        
        if solution is not None:
            # Verify solution with FK
            computed_pos, _ = self.fk.compute_fk(solution)
            error = np.linalg.norm(computed_pos - target_pos)
            
            if error < 0.01:  # 1cm tolerance
                return solution
        
        # Method 2: If geometric fails, try numerical optimization
        return self._numerical_ik(target_pos, wrist_angle, initial_guess)
    
    def _geometric_ik(self, target_pos: np.ndarray, wrist_angle: float) -> Optional[List[float]]:
        """
        Geometric/analytical IK solution
        
        This is specific to the 4-DOF arm configuration.
        """
        x, y, z = target_pos
        
        # Account for end effector length and base height
        # Work backwards from target to wrist center
        z_wrist = z - self.fk.ee_length
        z_relative = z_wrist - self.fk.z_offset - self.fk.link1_length
        
        # Joint 1: Base rotation (simple atan2)
        theta1 = np.arctan2(y, x)
        
        # Distance in XY plane to wrist
        r_xy = np.sqrt(x**2 + y**2)
        
        # Distance in the plane of joints 2 and 3
        # This is essentially a 2-link planar arm problem
        r = np.sqrt(r_xy**2 + z_relative**2)
        
        # Check if target is reachable
        l2 = self.fk.link2_length
        l3 = self.fk.link3_length
        
        if r > (l2 + l3) or r < abs(l2 - l3):
            return None  # Target unreachable
        
        # Joint 3: Elbow angle (law of cosines)
        cos_theta3 = (r**2 - l2**2 - l3**2) / (2 * l2 * l3)
        cos_theta3 = np.clip(cos_theta3, -1, 1)  # Ensure valid range
        
        # We can have elbow up or down - choose elbow down (negative angle)
        theta3 = -np.arccos(cos_theta3)
        
        # Joint 2: Shoulder angle
        # Angle to reach point in r-z plane
        alpha = np.arctan2(z_relative, r_xy)
        
        # Angle from link2 to link3
        beta = np.arctan2(l3 * np.sin(theta3), l2 + l3 * np.cos(theta3))
        
        theta2 = alpha - beta
        
        # Joint 4: Wrist rotation
        theta4 = wrist_angle
        
        solution = [theta1, theta2, theta3, theta4]
        
        # Check joint limits
        if self._check_joint_limits(solution):
            return solution
        
        return None
    
    def _numerical_ik(self, target_pos: np.ndarray, wrist_angle: float,
                      initial_guess: Optional[List[float]] = None) -> Optional[List[float]]:
        """
        Numerical IK using optimization (Jacobian-based or gradient descent)
        
        Args:
            target_pos: Desired position
            wrist_angle: Desired wrist angle
            initial_guess: Starting point for optimization
            
        Returns:
            Joint angles if converged, None otherwise
        """
        from scipy.optimize import minimize
        
        if initial_guess is None:
            initial_guess = [0, 0, 0, wrist_angle]
        
        def objective(joint_angles):
            """Objective function: distance to target"""
            pos, _ = self.fk.compute_fk(joint_angles)
            error = np.linalg.norm(pos - target_pos)
            
            # Add penalty for joint limit violations
            penalty = 0
            for i, (angle, (lower, upper)) in enumerate(zip(joint_angles, self.joint_limits)):
                if angle < lower or angle > upper:
                    penalty += 100 * (min(angle - lower, 0)**2 + max(0, angle - upper)**2)
            
            return error + penalty
        
        # Bounds for optimization
        bounds = self.joint_limits
        
        # Optimize
        result = minimize(objective, initial_guess, method='SLSQP', bounds=bounds,
                         options={'maxiter': 1000, 'ftol': 1e-6})
        
        if result.success and result.fun < 0.01:  # 1cm tolerance
            return result.x.tolist()
        
        return None
    
    def _check_joint_limits(self, joint_angles: List[float]) -> bool:
        """
        Check if joint angles are within limits
        
        Args:
            joint_angles: List of joint angles
            
        Returns:
            True if all angles within limits, False otherwise
        """
        for angle, (lower, upper) in zip(joint_angles, self.joint_limits):
            if angle < lower or angle > upper:
                return False
        return True
    
    def compute_ik_multiple_solutions(self, target_pos: np.ndarray, 
                                       wrist_angle: float = 0.0) -> List[List[float]]:
        """
        Find multiple IK solutions (elbow up/down configurations)
        
        Args:
            target_pos: Target position
            wrist_angle: Wrist rotation angle
            
        Returns:
            List of valid joint angle solutions
        """
        solutions = []
        
        # Try geometric solution (elbow down)
        sol1 = self._geometric_ik(target_pos, wrist_angle)
        if sol1 is not None:
            solutions.append(sol1)
        
        # Try elbow up configuration (modify geometric approach)
        # This would require modifying _geometric_ik to generate both
        
        return solutions


def test_inverse_kinematics():
    """Test the inverse kinematics solver"""
    fk = ForwardKinematics()
    ik = InverseKinematics()
    
    print("Testing Inverse Kinematics\n")
    
    # Test 1: Find IK for a known FK result
    print("Test 1: IK for known FK position")
    joint_angles_original = [0, np.pi/4, -np.pi/4, 0]
    target_pos, _ = fk.compute_fk(joint_angles_original)
    print(f"Target position: {target_pos}")
    
    solution = ik.compute_ik(target_pos, wrist_angle=0)
    if solution is not None:
        print(f"IK Solution: {solution}")
        computed_pos, _ = fk.compute_fk(solution)
        print(f"Computed position: {computed_pos}")
        print(f"Error: {np.linalg.norm(computed_pos - target_pos):.6f} m")
    else:
        print("No solution found")
    print()
    
    # Test 2: Specific target
    print("Test 2: Specific target position")
    target = np.array([0.3, 0.0, 0.5])
    print(f"Target: {target}")
    
    solution = ik.compute_ik(target)
    if solution is not None:
        print(f"IK Solution: {solution}")
        computed_pos, _ = fk.compute_fk(solution)
        print(f"Computed position: {computed_pos}")
        print(f"Error: {np.linalg.norm(computed_pos - target):.6f} m")
    else:
        print("No solution found (target may be unreachable)")
    print()


if __name__ == "__main__":
    test_inverse_kinematics()
