"""
Advanced Trajectory Planning for Robot Arm

Implements task-space and joint-space trajectory planning with
various interpolation methods and constraints.
"""

import numpy as np
from typing import List, Tuple, Optional
from .inverse_kinematics import InverseKinematics
from .forward_kinematics import ForwardKinematics


class AdvancedTrajectoryPlanner:
    """Advanced trajectory planner for task-space and joint-space"""
    
    def __init__(self, dt: float = 0.01):
        """
        Initialize advanced trajectory planner
        
        Args:
            dt: Time step for trajectory points
        """
        self.dt = dt
        self.ik = InverseKinematics()
        self.fk = ForwardKinematics()
        
    def joint_space_trajectory(self, start: np.ndarray, goal: np.ndarray,
                               duration: float, method: str = 'quintic') -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate smooth joint-space trajectory
        
        Args:
            start: Starting joint angles [4]
            goal: Goal joint angles [4]
            duration: Total duration in seconds
            method: 'linear', 'cubic', 'quintic', 'trapezoidal'
            
        Returns:
            Tuple of (positions [N, 4], velocities [N, 4], accelerations [N, 4])
        """
        n_points = int(duration / self.dt)
        
        positions = np.zeros((n_points, 4))
        velocities = np.zeros((n_points, 4))
        accelerations = np.zeros((n_points, 4))
        
        for i in range(n_points):
            t = i * self.dt
            s, s_dot, s_ddot = self._time_scaling(t, duration, method)
            
            # Position
            positions[i] = start + s * (goal - start)
            
            # Velocity
            velocities[i] = s_dot * (goal - start)
            
            # Acceleration
            accelerations[i] = s_ddot * (goal - start)
        
        return positions, velocities, accelerations
    
    def task_space_trajectory(self, start_pos: np.ndarray, goal_pos: np.ndarray,
                              duration: float, initial_joints: Optional[np.ndarray] = None,
                              method: str = 'quintic') -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate task-space trajectory (Cartesian straight line)
        
        Computes IK at each waypoint to get joint trajectories
        
        Args:
            start_pos: Starting Cartesian position [3]
            goal_pos: Goal Cartesian position [3]
            duration: Total duration
            initial_joints: Initial joint configuration for IK
            method: Interpolation method
            
        Returns:
            Tuple of (positions [N, 4], velocities [N, 4], accelerations [N, 4])
        """
        n_points = int(duration / self.dt)
        
        positions = np.zeros((n_points, 4))
        velocities = np.zeros((n_points, 4))
        accelerations = np.zeros((n_points, 4))
        
        # Current joint configuration for IK warm-start
        if initial_joints is None:
            current_joints = np.zeros(4)
        else:
            current_joints = initial_joints.copy()
        
        prev_joints = current_joints.copy()
        prev_prev_joints = current_joints.copy()
        
        for i in range(n_points):
            t = i * self.dt
            s, s_dot, s_ddot = self._time_scaling(t, duration, method)
            
            # Cartesian position along straight line
            cart_pos = start_pos + s * (goal_pos - start_pos)
            
            # Solve IK
            joint_solution = self.ik.compute_ik(cart_pos, initial_guess=current_joints.tolist())
            
            if joint_solution is None:
                # If IK fails, use previous solution
                joint_solution = current_joints
            
            positions[i] = joint_solution
            
            # Numerical differentiation for velocities and accelerations
            if i > 0:
                velocities[i] = (positions[i] - prev_joints) / self.dt
            
            if i > 1:
                accelerations[i] = (positions[i] - 2*prev_joints + prev_prev_joints) / (self.dt**2)
            
            # Update for next iteration
            prev_prev_joints = prev_joints.copy()
            prev_joints = positions[i].copy()
            current_joints = positions[i].copy()
        
        return positions, velocities, accelerations
    
    def circular_trajectory(self, center: np.ndarray, radius: float, 
                           normal: np.ndarray, duration: float,
                           initial_joints: Optional[np.ndarray] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate circular trajectory in task space
        
        Args:
            center: Circle center position [3]
            radius: Circle radius
            normal: Normal vector to circle plane [3]
            duration: Total duration
            initial_joints: Initial joint configuration
            
        Returns:
            Tuple of (positions [N, 4], velocities [N, 4], accelerations [N, 4])
        """
        n_points = int(duration / self.dt)
        
        # Orthonormal basis for circle plane
        normal = normal / np.linalg.norm(normal)
        
        # Find two perpendicular vectors in the plane
        if abs(normal[0]) < 0.9:
            u = np.cross(normal, np.array([1, 0, 0]))
        else:
            u = np.cross(normal, np.array([0, 1, 0]))
        u = u / np.linalg.norm(u)
        v = np.cross(normal, u)
        
        positions = np.zeros((n_points, 4))
        velocities = np.zeros((n_points, 4))
        accelerations = np.zeros((n_points, 4))
        
        if initial_joints is None:
            current_joints = np.zeros(4)
        else:
            current_joints = initial_joints.copy()
        
        prev_joints = current_joints.copy()
        prev_prev_joints = current_joints.copy()
        
        for i in range(n_points):
            t = i * self.dt
            theta = 2 * np.pi * t / duration  # Full circle
            
            # Circular path
            cart_pos = center + radius * (np.cos(theta) * u + np.sin(theta) * v)
            
            # Solve IK
            joint_solution = self.ik.compute_ik(cart_pos, initial_guess=current_joints.tolist())
            
            if joint_solution is None:
                joint_solution = current_joints
            
            positions[i] = joint_solution
            
            # Numerical differentiation
            if i > 0:
                velocities[i] = (positions[i] - prev_joints) / self.dt
            
            if i > 1:
                accelerations[i] = (positions[i] - 2*prev_joints + prev_prev_joints) / (self.dt**2)
            
            prev_prev_joints = prev_joints.copy()
            prev_joints = positions[i].copy()
            current_joints = positions[i].copy()
        
        return positions, velocities, accelerations
    
    def _time_scaling(self, t: float, T: float, method: str) -> Tuple[float, float, float]:
        """
        Compute time scaling function s(t) and its derivatives
        
        Args:
            t: Current time
            T: Total duration
            method: Scaling method
            
        Returns:
            Tuple of (s, s_dot, s_ddot)
        """
        # Normalize time to [0, 1]
        tau = np.clip(t / T, 0, 1)
        
        if method == 'linear':
            s = tau
            s_dot = 1.0 / T
            s_ddot = 0.0
            
        elif method == 'cubic':
            # 3τ² - 2τ³
            s = 3*tau**2 - 2*tau**3
            s_dot = (6*tau - 6*tau**2) / T
            s_ddot = (6 - 12*tau) / (T**2)
            
        elif method == 'quintic':
            # 10τ³ - 15τ⁴ + 6τ⁵
            s = 10*tau**3 - 15*tau**4 + 6*tau**5
            s_dot = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
            s_ddot = (60*tau - 180*tau**2 + 120*tau**3) / (T**2)
            
        elif method == 'trapezoidal':
            # Simplified trapezoidal profile
            t_accel = 0.25 * T
            t_decel = 0.75 * T
            
            if t < t_accel:
                s = 0.5 * (t / t_accel)**2
                s_dot = t / (t_accel * T)
                s_ddot = 1.0 / (t_accel * T)
            elif t < t_decel:
                s = 0.5 + (t - t_accel) / (t_decel - t_accel) * 0.5
                s_dot = 0.5 / (t_decel - t_accel)
                s_ddot = 0.0
            else:
                remaining = (T - t) / (T - t_decel)
                s = 1.0 - 0.5 * remaining**2
                s_dot = remaining / (T - t_decel)
                s_ddot = -1.0 / ((T - t_decel) * T)
        else:
            raise ValueError(f"Unknown method: {method}")
        
        return s, s_dot, s_ddot


def test_advanced_trajectory():
    """Test advanced trajectory planning"""
    planner = AdvancedTrajectoryPlanner(dt=0.01)
    
    print("=" * 70)
    print("ADVANCED TRAJECTORY PLANNING TESTS")
    print("=" * 70)
    
    # Test 1: Joint-space trajectory
    print("\nTest 1: Joint-Space Trajectory (Quintic)")
    print("-" * 70)
    start = np.array([0, 0, 0, 0])
    goal = np.array([np.pi/2, np.pi/4, -np.pi/4, 0])
    
    pos, vel, acc = planner.joint_space_trajectory(start, goal, duration=2.0, method='quintic')
    
    print(f"Generated {len(pos)} waypoints")
    print(f"Start position: {pos[0]}")
    print(f"End position: {pos[-1]}")
    print(f"Max velocity: {np.max(np.abs(vel)):.3f} rad/s")
    print(f"Max acceleration: {np.max(np.abs(acc)):.3f} rad/s²")
    
    # Test 2: Task-space trajectory
    print("\nTest 2: Task-Space Trajectory (Cartesian Line)")
    print("-" * 70)
    start_pos = np.array([0.2, 0.0, 0.7])
    goal_pos = np.array([0.3, 0.2, 0.5])
    
    pos, vel, acc = planner.task_space_trajectory(start_pos, goal_pos, duration=3.0)
    
    print(f"Generated {len(pos)} waypoints")
    print(f"Start joints: {pos[0]}")
    print(f"End joints: {pos[-1]}")
    
    # Verify endpoints
    fk = ForwardKinematics()
    achieved_start, _ = fk.compute_fk(pos[0])
    achieved_end, _ = fk.compute_fk(pos[-1])
    
    print(f"Start position error: {np.linalg.norm(achieved_start - start_pos)*1000:.3f} mm")
    print(f"End position error: {np.linalg.norm(achieved_end - goal_pos)*1000:.3f} mm")
    
    # Test 3: Circular trajectory
    print("\nTest 3: Circular Trajectory")
    print("-" * 70)
    center = np.array([0.3, 0.0, 0.5])
    radius = 0.1
    normal = np.array([0, 0, 1])  # XY plane
    
    pos, vel, acc = planner.circular_trajectory(center, radius, normal, duration=4.0)
    
    print(f"Generated {len(pos)} waypoints")
    print(f"Circle center: {center}")
    print(f"Radius: {radius}m")
    
    # Verify circularity
    positions_cart = []
    for joints in pos[::10]:  # Sample every 10th point
        p, _ = fk.compute_fk(joints)
        positions_cart.append(p)
    
    positions_cart = np.array(positions_cart)
    radii = np.linalg.norm(positions_cart - center, axis=1)
    print(f"Average radius: {np.mean(radii):.4f}m")
    print(f"Radius std dev: {np.std(radii):.4f}m")
    
    print("\n" + "=" * 70)


if __name__ == "__main__":
    test_advanced_trajectory()
