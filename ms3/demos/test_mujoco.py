"""
Demo script: Test MuJoCo Simulation
"""

import sys
import numpy as np
import time
from pathlib import Path

# Add paths
sys.path.insert(0, str(Path(__file__).parent.parent / "mujoco"))
sys.path.insert(0, str(Path(__file__).parent.parent / "ros2_ws" / "src" / "robot_arm_kinematics"))

from mujoco_simulator import MuJoCoSimulator
from robot_arm_kinematics.inverse_kinematics import InverseKinematics


def demo_mujoco_basic():
    """Basic MuJoCo simulation demo"""
    print("=" * 60)
    print("MuJoCo Basic Simulation Demo")
    print("=" * 60)
    
    # Get model path
    model_path = Path(__file__).parent.parent / "mujoco" / "robot_arm.xml"
    
    print(f"\nLoading model: {model_path}")
    
    # Create simulator
    sim = MuJoCoSimulator(str(model_path))
    
    # Reset to home
    sim.reset(np.array([0, 0, 0, 0]))
    
    print("\nStarting interactive simulation...")
    print("Control the robot using the viewer interface!")
    print("Close the window to exit.")
    
    # Run interactive
    sim.run_interactive()


def demo_mujoco_trajectory():
    """MuJoCo simulation with predefined trajectory"""
    print("=" * 60)
    print("MuJoCo Trajectory Execution Demo")
    print("=" * 60)
    
    # Get model path
    model_path = Path(__file__).parent.parent / "mujoco" / "robot_arm.xml"
    
    print(f"\nLoading model: {model_path}")
    
    # Create simulator
    sim = MuJoCoSimulator(str(model_path))
    
    # Define waypoints
    waypoints = [
        np.array([0, 0, 0, 0]),
        np.array([np.pi/2, 0, 0, 0]),
        np.array([np.pi/2, np.pi/4, 0, 0]),
        np.array([np.pi/2, np.pi/4, -np.pi/4, 0]),
        np.array([0, 0, 0, np.pi]),
        np.array([0, 0, 0, 0]),
    ]
    
    print("\nExecuting trajectory with waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  {i+1}. [{', '.join([f'{a:6.3f}' for a in wp])}]")
    
    trajectory = np.array(waypoints)
    
    print("\nStarting trajectory execution...")
    sim.run_trajectory(trajectory, duration=12.0, show_viewer=True)
    
    print("\nTrajectory complete!")


def demo_mujoco_ik_control():
    """MuJoCo simulation with IK-based control"""
    print("=" * 60)
    print("MuJoCo IK-based Control Demo")
    print("=" * 60)
    
    # Get model path
    model_path = Path(__file__).parent.parent / "mujoco" / "robot_arm.xml"
    
    print(f"\nLoading model: {model_path}")
    
    # Create simulator and IK solver
    sim = MuJoCoSimulator(str(model_path))
    ik = InverseKinematics()
    
    # Define Cartesian waypoints
    cartesian_targets = [
        np.array([0.0, 0.0, 0.8]),
        np.array([0.3, 0.0, 0.5]),
        np.array([0.2, 0.2, 0.4]),
        np.array([0.0, 0.3, 0.5]),
        np.array([-0.2, 0.2, 0.6]),
        np.array([0.0, 0.0, 0.8]),
    ]
    
    print("\nCartesian waypoints:")
    for i, target in enumerate(cartesian_targets):
        print(f"  {i+1}. [{target[0]:6.3f}, {target[1]:6.3f}, {target[2]:6.3f}]")
    
    # Compute IK for each target
    joint_trajectory = []
    
    print("\nComputing inverse kinematics...")
    for target in cartesian_targets:
        solution = ik.compute_ik(target)
        if solution is not None:
            joint_trajectory.append(solution)
            print(f"  Target {target} -> Joints: [{', '.join([f'{a:.3f}' for a in solution])}]")
        else:
            print(f"  Target {target} -> No solution!")
    
    if len(joint_trajectory) > 0:
        trajectory = np.array(joint_trajectory)
        print(f"\nExecuting {len(trajectory)} waypoints...")
        sim.run_trajectory(trajectory, duration=10.0, show_viewer=True)
        print("\nExecution complete!")
    else:
        print("\nNo valid trajectory computed!")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='MuJoCo Simulation Demos')
    parser.add_argument('--mode', type=str, default='basic',
                       choices=['basic', 'trajectory', 'ik'],
                       help='Demo mode to run')
    
    args = parser.parse_args()
    
    if args.mode == 'basic':
        demo_mujoco_basic()
    elif args.mode == 'trajectory':
        demo_mujoco_trajectory()
    elif args.mode == 'ik':
        demo_mujoco_ik_control()
