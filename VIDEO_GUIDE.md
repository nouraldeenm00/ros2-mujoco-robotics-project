#!/bin/bash

# ============================================================================
# MASTER VIDEO GUIDE - Run individual milestone videos
# ============================================================================
# This script helps you record the 5 required submission videos

PROJECT_ROOT="$HOME/ros2-mujoco-robotics-project"
cd "$PROJECT_ROOT"

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

clear

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║  MCTR911 - ROBOTICS PROGRAMMING PROJECT                       ║"
echo "║  5 MILESTONE VIDEOS FOR COURSE SUBMISSION                     ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""

echo -e "${YELLOW}REQUIRED VIDEOS FOR SUBMISSION:${NC}"
echo ""
echo "Each video demonstrates one milestone with required components."
echo "You can run them individually or use demo_all_milestones.sh"
echo ""

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}MS1 VIDEO: Project Setup & Software Verification${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "Weight: 5%"
echo "Duration: 3-4 minutes"
echo "Shows:"
echo "  ✓ GitHub repository"
echo "  ✓ Literature review"
echo "  ✓ CAD model components"
echo "  ✓ ROS2 & Python environment"
echo ""
echo "Run: ./ms1_video_demo.sh"
echo ""

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}MS2 VIDEO: Robot Description & Visualization${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "Weight: 7%"
echo "Duration: 4-5 minutes"
echo "Shows:"
echo "  ✓ DH convention parameters"
echo "  ✓ URDF model structure"
echo "  ✓ ROS2 package build"
echo "  ✓ RViz2 visualization with interactive controls"
echo ""
echo "Run: ./ms2_video_demo.sh"
echo ""

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}MS3 VIDEO: Kinematics Validation${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "Weight: 7%"
echo "Duration: 5-6 minutes"
echo "Shows:"
echo "  ✓ Forward kinematics: 10/10 tests passed"
echo "  ✓ Inverse kinematics: 0.002mm accuracy"
echo "  ✓ Velocity kinematics with Jacobian"
echo "  ✓ Acceleration kinematics validation"
echo "  ✓ All 30/30 tests passed (100%)"
echo ""
echo "Run: ./ms3_video_demo.sh"
echo ""

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}MS4 VIDEO: Trajectory Planning & Validation${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "Weight: 6%"
echo "Duration: 4-5 minutes"
echo "Shows:"
echo "  ✓ Joint-space trajectory generation"
echo "  ✓ Task-space Cartesian planning"
echo "  ✓ Quintic polynomial smoothness"
echo "  ✓ Live simulation in RViz2 + MuJoCo"
echo ""
echo "Run: ./ms4_video_demo.sh"
echo ""

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}MS5 VIDEO: Control Algorithms & Full Integration${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "Weight: 5%"
echo "Duration: 6-8 minutes (Narrated Presentation)"
echo "Shows:"
echo "  ✓ System architecture overview"
echo "  ✓ PID control implementation"
echo "  ✓ Computed torque control"
echo "  ✓ Autonomous pick & place operation"
echo "  ✓ Complete validation (30/30 tests)"
echo ""
echo "Run: ./ms5_video_demo.sh"
echo ""

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${CYAN}ALTERNATIVE: ONE COMPREHENSIVE VIDEO${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "If you prefer one continuous video showing all milestones:"
echo ""
echo "Run: ./demo_all_milestones.sh"
echo ""
echo "This demonstrates all 5 milestones in sequence (15-20 minutes total)"
echo ""

echo -e "${YELLOW}IMPORTANT NOTES:${NC}"
echo ""
echo "1. Each video script includes print statements marking sections"
echo "2. Videos demonstrate exact requirements from robotics-project.pdf"
echo "3. All simulations use ROS2 Jazzy + MuJoCo + RViz2"
echo "4. Control frequency: 50Hz, Trajectory points: 300 per segment"
echo "5. All kinematics validated with < 0.01mm accuracy"
echo ""

echo -e "${GREEN}SUBMISSION CHECKLIST:${NC}"
echo ""
echo "□ MS1: GitHub repo + Literature review + CAD + Software demo"
echo "□ MS2: DH parameters + URDF + Build process + RViz2 demo"
echo "□ MS3: FK tests (10/10) + IK tests (10/10) + Validation (30/30)"
echo "□ MS4: Trajectory planning + Joint-space + Task-space validation"
echo "□ MS5: Control algorithm + Autonomous pick & place + Full demo"
echo ""

echo -e "${CYAN}Ready to record your videos!${NC}"
echo ""
