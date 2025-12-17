"""
Trajectory Planner for 4-DOF Robot Arm

Generates smooth trajectories between waypoints
"""

import numpy as np
from typing import List, Tuple


class TrajectoryPlanner:
    """Trajectory planning for robot arm"""
    
    def __init__(self, dt: float = 0.01):
        """
        Initialize trajectory planner
        
        Args:
            dt: Time step for trajectory points
        """
        self.dt = dt
    
    def linear_joint_trajectory(self, start: np.ndarray, goal: np.ndarray, 
                                duration: float) -> np.ndarray:
        """
        Generate linear interpolation trajectory in joint space
        
        Args:
            start: Starting joint angles [4]
            goal: Goal joint angles [4]
            duration: Total duration in seconds
            
        Returns:
            Trajectory array of shape (n_points, 4)
        """
        n_points = int(duration / self.dt)
        trajectory = np.zeros((n_points, 4))
        
        for i in range(n_points):
            alpha = i / (n_points - 1)  # 0 to 1
            trajectory[i] = start + alpha * (goal - start)
        
        return trajectory
    
    def smooth_joint_trajectory(self, start: np.ndarray, goal: np.ndarray,
                                duration: float, profile: str = 'cubic') -> np.ndarray:
        """
        Generate smooth trajectory with velocity constraints
        
        Args:
            start: Starting joint angles
            goal: Goal joint angles
            duration: Total duration
            profile: Velocity profile ('cubic', 'quintic', 'trapezoidal')
            
        Returns:
            Trajectory array of shape (n_points, 4)
        """
        n_points = int(duration / self.dt)
        trajectory = np.zeros((n_points, 4))
        
        if profile == 'cubic':
            # Cubic polynomial (zero velocity at endpoints)
            for i in range(n_points):
                s = i / (n_points - 1)  # 0 to 1
                # Cubic: 3s^2 - 2s^3
                alpha = 3*s**2 - 2*s**3
                trajectory[i] = start + alpha * (goal - start)
                
        elif profile == 'quintic':
            # Quintic polynomial (zero velocity and acceleration at endpoints)
            for i in range(n_points):
                s = i / (n_points - 1)
                # Quintic: 10s^3 - 15s^4 + 6s^5
                alpha = 10*s**3 - 15*s**4 + 6*s**5
                trajectory[i] = start + alpha * (goal - start)
                
        else:  # trapezoidal
            # Simplified trapezoidal profile
            accel_time = duration * 0.25
            decel_time = duration * 0.75
            
            for i in range(n_points):
                t = i * self.dt
                
                if t < accel_time:
                    # Acceleration phase
                    alpha = 0.5 * (t / accel_time)**2
                elif t < decel_time:
                    # Constant velocity phase
                    alpha = 0.5 + (t - accel_time) / (decel_time - accel_time) * 0.5
                else:
                    # Deceleration phase
                    alpha = 1.0 - 0.5 * ((duration - t) / (duration - decel_time))**2
                
                trajectory[i] = start + alpha * (goal - start)
        
        return trajectory
    
    def multi_waypoint_trajectory(self, waypoints: List[np.ndarray],
                                  durations: List[float],
                                  profile: str = 'cubic') -> np.ndarray:
        """
        Generate trajectory through multiple waypoints
        
        Args:
            waypoints: List of joint angle arrays
            durations: Duration for each segment
            profile: Velocity profile to use
            
        Returns:
            Combined trajectory array
        """
        if len(waypoints) != len(durations) + 1:
            raise ValueError("Number of waypoints should be one more than durations")
        
        segments = []
        for i in range(len(durations)):
            segment = self.smooth_joint_trajectory(
                waypoints[i], waypoints[i+1], durations[i], profile
            )
            segments.append(segment)
        
        return np.vstack(segments)


def test_trajectory_planner():
    """Test trajectory planning"""
    planner = TrajectoryPlanner(dt=0.01)
    
    # Test 1: Linear trajectory
    print("Test 1: Linear trajectory")
    start = np.array([0, 0, 0, 0])
    goal = np.array([np.pi/2, np.pi/4, -np.pi/4, np.pi/2])
    
    traj = planner.linear_joint_trajectory(start, goal, duration=2.0)
    print(f"Generated {len(traj)} points")
    print(f"Start: {traj[0]}")
    print(f"End: {traj[-1]}")
    print()
    
    # Test 2: Smooth trajectory
    print("Test 2: Smooth cubic trajectory")
    traj_smooth = planner.smooth_joint_trajectory(start, goal, duration=2.0, profile='cubic')
    print(f"Generated {len(traj_smooth)} points")
    print(f"Start: {traj_smooth[0]}")
    print(f"End: {traj_smooth[-1]}")
    print()


if __name__ == "__main__":
    test_trajectory_planner()
