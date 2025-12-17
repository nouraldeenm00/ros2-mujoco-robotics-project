"""
Demo script: Test Forward and Inverse Kinematics
"""

import sys
import numpy as np
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "ros2_ws" / "src" / "robot_arm_kinematics"))

from robot_arm_kinematics.forward_kinematics import ForwardKinematics
from robot_arm_kinematics.inverse_kinematics import InverseKinematics


def demo_kinematics():
    """Demonstrate forward and inverse kinematics"""
    
    print("=" * 60)
    print("4-DOF Robot Arm Kinematics Demo")
    print("=" * 60)
    print()
    
    fk = ForwardKinematics()
    ik = InverseKinematics()
    
    # Test 1: Forward Kinematics
    print("Test 1: Forward Kinematics")
    print("-" * 60)
    
    test_configs = [
        ([0, 0, 0, 0], "Home position"),
        ([np.pi/4, 0, 0, 0], "45° base rotation"),
        ([0, np.pi/4, 0, 0], "45° shoulder"),
        ([0, 0, np.pi/4, 0], "45° elbow"),
        ([np.pi/2, np.pi/4, -np.pi/4, 0], "Complex pose"),
    ]
    
    for angles, description in test_configs:
        pos, rot = fk.compute_fk(angles)
        print(f"\n{description}:")
        print(f"  Joint angles: [{', '.join([f'{a:6.3f}' for a in angles])}]")
        print(f"  End-effector: [{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}]")
    
    print()
    print("=" * 60)
    print()
    
    # Test 2: Inverse Kinematics
    print("Test 2: Inverse Kinematics")
    print("-" * 60)
    
    test_targets = [
        np.array([0.3, 0.0, 0.5]),
        np.array([0.2, 0.2, 0.4]),
        np.array([0.0, 0.3, 0.6]),
        np.array([-0.2, 0.1, 0.45]),
    ]
    
    for target in test_targets:
        print(f"\nTarget: [{target[0]:6.3f}, {target[1]:6.3f}, {target[2]:6.3f}]")
        
        solution = ik.compute_ik(target)
        
        if solution is not None:
            print(f"  IK Solution: [{', '.join([f'{a:6.3f}' for a in solution])}]")
            
            # Verify with FK
            computed_pos, _ = fk.compute_fk(solution)
            error = np.linalg.norm(computed_pos - target)
            print(f"  Achieved:    [{computed_pos[0]:6.3f}, {computed_pos[1]:6.3f}, {computed_pos[2]:6.3f}]")
            print(f"  Error:       {error*1000:.3f} mm")
        else:
            print("  No solution found (target unreachable)")
    
    print()
    print("=" * 60)
    print()
    
    # Test 3: Round-trip test
    print("Test 3: FK -> IK Round-trip Test")
    print("-" * 60)
    
    original_angles = [np.pi/3, np.pi/6, -np.pi/6, np.pi/4]
    print(f"\nOriginal angles: [{', '.join([f'{a:6.3f}' for a in original_angles])}]")
    
    # Forward
    target_pos, _ = fk.compute_fk(original_angles)
    print(f"FK position:     [{target_pos[0]:6.3f}, {target_pos[1]:6.3f}, {target_pos[2]:6.3f}]")
    
    # Inverse
    recovered_angles = ik.compute_ik(target_pos, wrist_angle=original_angles[3])
    
    if recovered_angles is not None:
        print(f"IK solution:     [{', '.join([f'{a:6.3f}' for a in recovered_angles])}]")
        
        # Verify
        final_pos, _ = fk.compute_fk(recovered_angles)
        print(f"Final position:  [{final_pos[0]:6.3f}, {final_pos[1]:6.3f}, {final_pos[2]:6.3f}]")
        
        pos_error = np.linalg.norm(final_pos - target_pos)
        print(f"\nPosition error: {pos_error*1000:.3f} mm")
    else:
        print("IK failed!")
    
    print()
    print("=" * 60)


if __name__ == "__main__":
    demo_kinematics()
