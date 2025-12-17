"""
Control Algorithms for Robot Arm

Implements PID control, computed torque control, and trajectory tracking.
"""

import numpy as np
from typing import Optional, Tuple
from .forward_kinematics import ForwardKinematics
from .velocity_kinematics import VelocityKinematics


class PIDController:
    """PID controller for joint-level control"""
    
    def __init__(self, kp: np.ndarray, ki: np.ndarray, kd: np.ndarray):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gains [4]
            ki: Integral gains [4]
            kd: Derivative gains [4]
        """
        self.kp = np.array(kp)
        self.ki = np.array(ki)
        self.kd = np.array(kd)
        
        # Integral error accumulator
        self.integral_error = np.zeros(4)
        
        # Previous error for derivative
        self.prev_error = np.zeros(4)
        
    def compute(self, desired: np.ndarray, actual: np.ndarray,
                dt: float, desired_vel: Optional[np.ndarray] = None,
                actual_vel: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Compute PID control output
        
        u = Kp*e + Ki*∫e + Kd*ė
        
        Args:
            desired: Desired joint positions [4]
            actual: Actual joint positions [4]
            dt: Time step
            desired_vel: Desired velocities (optional)
            actual_vel: Actual velocities (optional)
            
        Returns:
            Control torques [4]
        """
        # Position error
        error = desired - actual
        
        # Integral of error (with anti-windup)
        self.integral_error += error * dt
        self.integral_error = np.clip(self.integral_error, -10, 10)
        
        # Derivative of error
        if desired_vel is not None and actual_vel is not None:
            # Use velocity feedback if available
            error_derivative = desired_vel - actual_vel
        else:
            # Numerical derivative of position error
            error_derivative = (error - self.prev_error) / dt
        
        # PID control law
        control = (self.kp * error + 
                  self.ki * self.integral_error + 
                  self.kd * error_derivative)
        
        self.prev_error = error.copy()
        
        return control
    
    def reset(self):
        """Reset integral and derivative terms"""
        self.integral_error = np.zeros(4)
        self.prev_error = np.zeros(4)


class ComputedTorqueController:
    """Computed torque control (inverse dynamics control)"""
    
    def __init__(self):
        """Initialize computed torque controller"""
        self.fk = ForwardKinematics()
        self.vk = VelocityKinematics()
        
        # Simplified robot parameters (in real application, use actual inertia matrix)
        self.mass_matrix_diag = np.array([1.0, 0.5, 0.4, 0.2])  # Simplified
        
    def compute(self, desired_pos: np.ndarray, desired_vel: np.ndarray,
                desired_acc: np.ndarray, actual_pos: np.ndarray,
                actual_vel: np.ndarray, kp: np.ndarray, kd: np.ndarray) -> np.ndarray:
        """
        Compute control torques using computed torque method
        
        τ = M(q)[q̈_d + Kd*ė + Kp*e] + C(q,q̇) + g(q)
        
        Args:
            desired_pos: Desired positions [4]
            desired_vel: Desired velocities [4]
            desired_acc: Desired accelerations [4]
            actual_pos: Actual positions [4]
            actual_vel: Actual velocities [4]
            kp: Position feedback gains [4]
            kd: Velocity feedback gains [4]
            
        Returns:
            Control torques [4]
        """
        # Position and velocity errors
        e_pos = desired_pos - actual_pos
        e_vel = desired_vel - actual_vel
        
        # Desired acceleration with feedback
        acc_feedback = desired_acc + kd * e_vel + kp * e_pos
        
        # Simplified mass matrix (diagonal)
        # In real application, would compute full M(q)
        M = np.diag(self.mass_matrix_diag)
        
        # Coriolis and centrifugal terms (simplified as damping)
        C = 0.1 * actual_vel
        
        # Gravity compensation (simplified)
        g = self._compute_gravity(actual_pos)
        
        # Computed torque
        torque = M @ acc_feedback + C + g
        
        return torque
    
    def _compute_gravity(self, q: np.ndarray) -> np.ndarray:
        """
        Compute gravity torques (simplified)
        
        Args:
            q: Joint angles
            
        Returns:
            Gravity torques [4]
        """
        # Simplified gravity model
        # Joint 1 (base rotation): no gravity effect
        # Joints 2-3: gravity acts on links
        g = 9.81
        
        tau_g = np.zeros(4)
        tau_g[0] = 0  # Base rotation
        tau_g[1] = g * 0.5 * np.cos(q[1])  # Shoulder
        tau_g[2] = g * 0.3 * np.cos(q[1] + q[2])  # Elbow
        tau_g[3] = 0  # Wrist rotation
        
        return tau_g


class TrajectoryTracker:
    """High-level trajectory tracking controller"""
    
    def __init__(self, control_type: str = 'pid'):
        """
        Initialize trajectory tracker
        
        Args:
            control_type: 'pid' or 'computed_torque'
        """
        self.control_type = control_type
        
        if control_type == 'pid':
            # Tuned PID gains
            self.controller = PIDController(
                kp=np.array([100.0, 100.0, 80.0, 50.0]),
                ki=np.array([1.0, 1.0, 1.0, 0.5]),
                kd=np.array([20.0, 20.0, 15.0, 10.0])
            )
        elif control_type == 'computed_torque':
            self.controller = ComputedTorqueController()
            self.kp = np.array([100.0, 100.0, 80.0, 50.0])
            self.kd = np.array([20.0, 20.0, 15.0, 10.0])
        else:
            raise ValueError(f"Unknown control type: {control_type}")
    
    def track(self, trajectory_pos: np.ndarray, trajectory_vel: np.ndarray,
              trajectory_acc: np.ndarray, current_pos: np.ndarray,
              current_vel: np.ndarray, dt: float, step: int) -> np.ndarray:
        """
        Track a trajectory
        
        Args:
            trajectory_pos: Position trajectory [N, 4]
            trajectory_vel: Velocity trajectory [N, 4]
            trajectory_acc: Acceleration trajectory [N, 4]
            current_pos: Current position [4]
            current_vel: Current velocity [4]
            dt: Time step
            step: Current step index
            
        Returns:
            Control torques [4]
        """
        if step >= len(trajectory_pos):
            step = len(trajectory_pos) - 1
        
        desired_pos = trajectory_pos[step]
        desired_vel = trajectory_vel[step]
        desired_acc = trajectory_acc[step]
        
        if self.control_type == 'pid':
            torque = self.controller.compute(
                desired_pos, current_pos, dt,
                desired_vel, current_vel
            )
        else:  # computed_torque
            torque = self.controller.compute(
                desired_pos, desired_vel, desired_acc,
                current_pos, current_vel,
                self.kp, self.kd
            )
        
        return torque
    
    def reset(self):
        """Reset controller state"""
        if self.control_type == 'pid':
            self.controller.reset()


def test_controllers():
    """Test control algorithms"""
    print("=" * 70)
    print("CONTROL ALGORITHMS TESTING")
    print("=" * 70)
    
    # Test 1: PID Controller
    print("\nTest 1: PID Controller")
    print("-" * 70)
    
    pid = PIDController(
        kp=np.array([100, 100, 80, 50]),
        ki=np.array([1, 1, 1, 0.5]),
        kd=np.array([20, 20, 15, 10])
    )
    
    # Simulate step response
    desired = np.array([1.0, 0.5, -0.3, 0.2])
    actual = np.zeros(4)
    actual_vel = np.zeros(4)
    dt = 0.01
    
    print(f"Desired: {desired}")
    
    for i in range(200):
        control = pid.compute(desired, actual, dt)
        
        # Simple dynamics simulation
        actual_vel += control * dt * 0.1  # Simplified
        actual += actual_vel * dt
        
        if i % 50 == 0:
            error = np.linalg.norm(desired - actual)
            print(f"Step {i:3d}: Position error = {error:.6f}")
    
    final_error = np.linalg.norm(desired - actual)
    print(f"Final error: {final_error:.6f}")
    
    # Test 2: Computed Torque Controller
    print("\nTest 2: Computed Torque Controller")
    print("-" * 70)
    
    ctc = ComputedTorqueController()
    
    desired_pos = np.array([0.5, 0.3, -0.2, 0.1])
    desired_vel = np.array([0.1, 0.05, -0.1, 0.0])
    desired_acc = np.array([0.01, -0.01, 0.02, 0.0])
    
    actual_pos = np.array([0.4, 0.25, -0.15, 0.05])
    actual_vel = np.array([0.08, 0.04, -0.08, 0.01])
    
    kp = np.array([100, 100, 80, 50])
    kd = np.array([20, 20, 15, 10])
    
    torque = ctc.compute(desired_pos, desired_vel, desired_acc,
                        actual_pos, actual_vel, kp, kd)
    
    print(f"Desired position: {desired_pos}")
    print(f"Actual position:  {actual_pos}")
    print(f"Position error:   {desired_pos - actual_pos}")
    print(f"Control torque:   {torque}")
    
    print("\n" + "=" * 70)


if __name__ == "__main__":
    test_controllers()
