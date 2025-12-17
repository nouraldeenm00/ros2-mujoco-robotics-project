"""
MuJoCo Simulator Interface for 4-DOF Robot Arm

This module provides a Python interface to control and visualize
the robot arm in MuJoCo.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
from pathlib import Path


class MuJoCoSimulator:
    """MuJoCo simulation environment for the robot arm"""
    
    def __init__(self, model_path: str):
        """
        Initialize MuJoCo simulator
        
        Args:
            model_path: Path to MuJoCo XML model file
        """
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        # Load model and create data
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        
        # Joint names
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.n_joints = len(self.joint_names)
        
        # Get joint IDs
        self.joint_ids = [self.model.joint(name).id for name in self.joint_names]
        
        # Viewer
        self.viewer = None
        
    def reset(self, joint_angles: np.ndarray = None):
        """
        Reset simulation to initial state
        
        Args:
            joint_angles: Initial joint angles (if None, use zeros)
        """
        mujoco.mj_resetData(self.model, self.data)
        
        if joint_angles is not None:
            self.set_joint_positions(joint_angles)
        
        mujoco.mj_forward(self.model, self.data)
    
    def set_joint_positions(self, joint_angles: np.ndarray):
        """
        Set joint positions
        
        Args:
            joint_angles: Array of 4 joint angles in radians
        """
        for i, angle in enumerate(joint_angles):
            self.data.qpos[i] = angle
    
    def get_joint_positions(self) -> np.ndarray:
        """
        Get current joint positions
        
        Returns:
            Array of 4 joint angles in radians
        """
        return self.data.qpos[:self.n_joints].copy()
    
    def get_joint_velocities(self) -> np.ndarray:
        """
        Get current joint velocities
        
        Returns:
            Array of 4 joint velocities in rad/s
        """
        return self.data.qvel[:self.n_joints].copy()
    
    def get_end_effector_position(self) -> np.ndarray:
        """
        Get end effector position
        
        Returns:
            3D position [x, y, z]
        """
        ee_site_id = self.model.site('ee_tip').id
        return self.data.site_xpos[ee_site_id].copy()
    
    def set_control(self, joint_angles: np.ndarray):
        """
        Set control inputs (target joint angles for position control)
        
        Args:
            joint_angles: Desired joint angles
        """
        self.data.ctrl[:self.n_joints] = joint_angles
    
    def step(self, n_steps: int = 1):
        """
        Step the simulation forward
        
        Args:
            n_steps: Number of simulation steps
        """
        for _ in range(n_steps):
            mujoco.mj_step(self.model, self.data)
    
    def run_interactive(self):
        """
        Run interactive viewer
        """
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            self.viewer = viewer
            
            # Enable joint controls in viewer
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
            
            print("MuJoCo Interactive Viewer")
            print("Close the viewer window to exit")
            print("Use the sliders or double-click on the robot to control joints")
            
            while viewer.is_running():
                step_start = time.time()
                
                # Step simulation
                mujoco.mj_step(self.model, self.data)
                
                # Update viewer
                viewer.sync()
                
                # Maintain real-time
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
    
    def run_trajectory(self, trajectory: np.ndarray, duration: float = 5.0, 
                       show_viewer: bool = True):
        """
        Execute a joint trajectory
        
        Args:
            trajectory: Array of shape (n_waypoints, 4) with joint angles
            duration: Total duration for trajectory execution
            show_viewer: Whether to show visualization
        """
        n_waypoints = len(trajectory)
        dt = self.model.opt.timestep
        steps_per_waypoint = int((duration / n_waypoints) / dt)
        
        if show_viewer:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                for waypoint in trajectory:
                    self.set_control(waypoint)
                    
                    for _ in range(steps_per_waypoint):
                        mujoco.mj_step(self.model, self.data)
                        viewer.sync()
                        time.sleep(dt)
        else:
            for waypoint in trajectory:
                self.set_control(waypoint)
                self.step(steps_per_waypoint)


def demo_simulation():
    """Demo the MuJoCo simulation"""
    import os
    
    # Get model path
    script_dir = Path(__file__).parent.parent
    model_path = script_dir / "mujoco" / "robot_arm.xml"
    
    print(f"Loading model from: {model_path}")
    
    # Create simulator
    sim = MuJoCoSimulator(str(model_path))
    
    # Reset to home position
    sim.reset(np.array([0, 0, 0, 0]))
    
    print("Starting interactive simulation...")
    print("Try controlling the joints!")
    
    # Run interactive
    sim.run_interactive()


if __name__ == "__main__":
    demo_simulation()
