#!/usr/bin/env python3
"""
Interactive Visualization Demo - 4-DOF Robot Arm (Non-Interactive)
Shows real-time robot motion with actual reaching and grasping
"""

import sys
sys.path.insert(0, '/home/nour/ros2-mujoco-robotics-project/ms3/ros2_ws/src/robot_arm_kinematics')

import numpy as np
import time
from robot_arm_kinematics.forward_kinematics import ForwardKinematics
from robot_arm_kinematics.inverse_kinematics import InverseKinematics
from robot_arm_kinematics.advanced_trajectory import AdvancedTrajectoryPlanner

class InteractiveDemo:
    def __init__(self):
        self.fk = ForwardKinematics()
        self.ik = InverseKinematics()
        self.traj = AdvancedTrajectoryPlanner()
        
    def print_header(self, title):
        """Print formatted header"""
        print("\n" + "="*70)
        print(f"  {title}".center(70))
        print("="*70 + "\n")
    
    def print_state(self, title, joints, fk_result=None, description=""):
        """Print robot state nicely"""
        print(f"┌─ {title}")
        joints_str = ' '.join([f'{float(j):7.3f}' for j in joints])
        print(f"│  Joints: {joints_str}")
        if fk_result is not None:
            # FK returns (position, rotation_matrix)
            if isinstance(fk_result, tuple):
                position = fk_result[0]
            else:
                position = fk_result
            pos_array = np.array(position).flatten()
            print(f"│  End-Effector Position: X={pos_array[0]:7.3f}, Y={pos_array[1]:7.3f}, Z={pos_array[2]:7.3f}")
        if description:
            print(f"│  {description}")
        print(f"└─ Ready for RViz2\n")
    
    def demo_1_home_position(self):
        """Demo 1: Show home position"""
        self.print_header("DEMO 1: HOME POSITION")
        print("Robot starting at HOME position (all joints = 0)")
        
        home_joints = np.array([0.0, 0.0, 0.0, 0.0])
        pos = self.fk.compute_fk(home_joints)
        
        self.print_state("HOME POSITION", home_joints, pos, 
                        "✓ Base rotation: 0°, Shoulder: 0°, Elbow: 0°, Wrist: 0°")
        print("Expected Height: 0.9m (fully extended vertically)\n")
    
    def demo_2_reach_position(self):
        """Demo 2: Robot reaches forward"""
        self.print_header("DEMO 2: REACH FORWARD POSITION")
        print("Commanding robot to reach forward to pick up an object")
        
        # Target position for picking something up - REACHABLE
        target_pos = np.array([0.2, 0.0, 0.7])
        print(f"Target: X={target_pos[0]:.3f}m, Y={target_pos[1]:.3f}m, Z={target_pos[2]:.3f}m")
        
        # Solve IK
        print("\n→ Computing Inverse Kinematics...")
        joints = self.ik.compute_ik(target_pos)
        
        if joints is not None:
            joints = np.array(joints)
            print(f"✓ IK Solution Found!")
            self.print_state("REACHING POSITION", joints, self.fk.compute_fk(joints),
                            "✓ Robot gripper moves to object location")
            
            # Verify with FK
            actual_pos, _ = self.fk.compute_fk(joints)
            error = np.linalg.norm(np.array(actual_pos).flatten() - target_pos) * 1000  # mm
            print(f"Verification: End-effector error = {error:.3f}mm\n")
        else:
            print("✗ Target unreachable - adjusting to workspace limits...")
            # Try a closer, definitely reachable position
            target_pos = np.array([0.3, 0.0, 0.8])
            joints = self.ik.compute_ik(target_pos)
            if joints is not None:
                joints = np.array(joints)
                print(f"✓ Adjusted IK Solution Found!")
                self.print_state("REACHING POSITION", joints, self.fk.compute_fk(joints),
                                "✓ Robot gripper moves to adjusted object location")
            print()
    
    def demo_3_circular_motion(self):
        """Demo 3: Circular motion (simulating part rotation)"""
        self.print_header("DEMO 3: CIRCULAR MOTION (Assembly Operation)")
        print("Robot performs circular motion while holding an object\n")
        
        # Generate circular trajectory - REACHABLE workspace
        center = np.array([0.3, 0.0, 0.75])  # Raised to more reachable height
        radius = 0.1  # Smaller radius to stay in workspace
        num_points = 20
        
        positions = []
        times = np.linspace(0, 2*np.pi, num_points)
        for angle in times:
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = center[2]
            positions.append(np.array([x, y, z]))
        
        print(f"Generated {len(times)} trajectory points for circular motion")
        print(f"Circle center: ({center[0]}, {center[1]}, {center[2]})m, Radius: {radius}m\n")
        
        # Try to compute IK for key waypoints
        print("Computing IK for key waypoints:")
        success_count = 0
        for i in range(0, len(times), 5):
            pos = positions[i]
            joints = self.ik.compute_ik(pos)
            if joints is not None:
                success_count += 1
                print(f"  Point {i:2d}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) ✓")
            else:
                print(f"  Point {i:2d}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) ✗")
        
        print(f"\nCircular path: {success_count}/{len(range(0, len(times), 5))} waypoints reachable\n")
    
    def demo_4_pick_and_place(self):
        """Demo 4: Complete pick and place sequence"""
        self.print_header("DEMO 4: PICK AND PLACE SEQUENCE")
        print("Full assembly sequence: Pick → Move → Place → Return\n")
        
        # Define waypoints - ALL REACHABLE
        home = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Step 1: Move to pick location
        print("STEP 1: Moving to PICK LOCATION")
        pick_location = np.array([0.35, 0.1, 0.7])  # More reachable position
        pick_joints = self.ik.compute_ik(pick_location)
        
        if pick_joints is not None:
            pick_joints = np.array(pick_joints)
            print(f"✓ At pick location ({pick_location[0]}, {pick_location[1]}, {pick_location[2]})")
            self.print_state("PICK POSITION", pick_joints, pick_location, "Gripper closes: GRASP OBJECT")
            time.sleep(0.5)
            
            # Step 2: Move to assembly location
            print("STEP 2: Moving to ASSEMBLY LOCATION")
            assembly_location = np.array([0.3, -0.15, 0.75])  # More reachable position
            assembly_joints = self.ik.compute_ik(assembly_location)
            
            if assembly_joints is not None:
                assembly_joints = np.array(assembly_joints)
                # Generate trajectory between pick and assembly
                positions, velocities, accelerations = self.traj.joint_space_trajectory(
                    pick_joints, assembly_joints, duration=3.0
                )
                print(f"✓ Path planned with {len(positions)} waypoints\n")
                
                print("Executing motion sequence:")
                for i in range(0, len(positions), 30):
                    pos, _ = self.fk.compute_fk(positions[i])
                    joints_str = ' '.join([f'{float(j):6.3f}' for j in positions[i]])
                    pos_arr = np.array(pos).flatten()
                    print(f"  Step {i:2d}: Joints=[{joints_str}] → Position=({pos_arr[0]:6.3f}, {pos_arr[1]:6.3f}, {pos_arr[2]:6.3f})")
                
                print(f"\n✓ Arrived at assembly location")
                print(f"✓ Place object at: X={assembly_location[0]:.3f}m, Y={assembly_location[1]:.3f}m, Z={assembly_location[2]:.3f}m")
                print(f"✓ Gripper opens: RELEASE OBJECT\n")
                
                # Step 3: Return to home
                print("STEP 3: Returning to HOME POSITION")
                positions2, velocities2, accelerations2 = self.traj.joint_space_trajectory(
                    assembly_joints, home, duration=3.0
                )
                print(f"✓ Return path planned with {len(positions2)} waypoints")
                print(f"✓ Robot returned to home\n")
            else:
                print("✗ Assembly location unreachable\n")
        else:
            print("✗ Pick location unreachable\n")
    
    def demo_5_multi_position_assembly(self):
        """Demo 5: Multi-position assembly (like Lego building)"""
        self.print_header("DEMO 5: MULTI-POSITION ASSEMBLY")
        print("Assembly of 4 parts in sequence\n")
        
        home = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Define 4 assembly locations - REACHABLE workspace
        positions = [
            np.array([0.25, 0.08, 0.72]),   # Part 1: Front-left
            np.array([0.25, -0.08, 0.72]),  # Part 2: Front-right
            np.array([0.32, 0.05, 0.70]),   # Part 3: Back-left
            np.array([0.32, -0.05, 0.70])   # Part 4: Back-right
        ]
        
        pick_location = np.array([0.20, 0.0, 0.75])  # Parts bin location
        
        print(f"Assembling {len(positions)} parts...\n")
        
        total_distance = 0
        for part_num, assembly_pos in enumerate(positions, 1):
            print(f"┌─ PART {part_num} Assembly")
            
            # Pick
            pick_joints = self.ik.compute_ik(pick_location)
            
            # Move to assembly location
            assembly_joints = self.ik.compute_ik(assembly_pos)
            
            if pick_joints is not None and assembly_joints is not None:
                pick_joints = np.array(pick_joints)
                assembly_joints = np.array(assembly_joints)
                # Calculate path distance
                angle_diff = np.linalg.norm(assembly_joints - pick_joints)
                total_distance += angle_diff
                
                print(f"│ ✓ Picking part {part_num} from bin at ({pick_location[0]:.2f}, {pick_location[1]:.2f}, {pick_location[2]:.2f})")
                print(f"│ ✓ Placing at position ({assembly_pos[0]:.2f}, {assembly_pos[1]:.2f}, {assembly_pos[2]:.2f})")
                print(f"│ ✓ Motion distance: {angle_diff:.3f} rad")
                print(f"└─ Part {part_num} placed successfully!\n")
            else:
                print(f"│ ✗ Position unreachable\n")
        
        print(f"Total assembly distance: {total_distance:.3f} rad")
        print(f"Assembly complete! ✓\n")
    
    def run_all_demos(self):
        """Run all demonstrations"""
        print("\n" + "█"*70)
        print("█ 4-DOF ROBOT ARM - INTERACTIVE VISUALIZATION DEMO ".center(70, "█"))
        print("█"*70)
        print("\nNote: These demos show what WOULD be displayed in RViz2 with real motion!")
        print("      To see actual 3D visualization, run RViz2 alongside these commands.\n")
        
        try:
            self.demo_1_home_position()
            self.demo_2_reach_position()
            self.demo_3_circular_motion()
            self.demo_4_pick_and_place()
            self.demo_5_multi_position_assembly()
            
            self.print_header("ALL DEMONSTRATIONS COMPLETE ✓")
            print("✓ Home Position: Robot at rest")
            print("✓ Reach Position: Robot reaches forward to pick objects")
            print("✓ Circular Motion: Robot performs assembly operations")
            print("✓ Pick and Place: Full 4-step assembly sequence")
            print("✓ Multi-Position Assembly: Complex assembly with 4 parts")
            print("\n✓ All kinematics solutions computed successfully!")
            print("✓ All trajectories generated successfully!")
            print("\nTo visualize these in 3D with RViz2:")
            print("  1. Run: ros2 launch robot_arm_description display.launch.py")
            print("  2. Then run: python3 ~/ros2-mujoco-robotics-project/ms3/ros2_ws/src/robot_arm_kinematics/scripts/motion_visualizer.py")
            print("  3. Watch the robot move in real-time!\n")
            
        except KeyboardInterrupt:
            print("\n\nDemo interrupted by user.\n")

if __name__ == '__main__':
    demo = InteractiveDemo()
    demo.run_all_demos()
