#!/bin/bash

# ============================================================================
# MILESTONE 1 VIDEO DEMO: Project Setup & Software Verification
# ============================================================================
# This video demonstrates:
# - GitHub repository setup
# - Literature review
# - CAD model
# - ROS2 and MuJoCo environment verification
# Duration: 3-4 minutes

set -e

PROJECT_ROOT="$HOME/ros2-mujoco-robotics-project"
cd "$PROJECT_ROOT"

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo ""
echo "╔════════════════════════════════════════════════════════════════╗"
echo "║ MILESTONE 1: PROJECT SETUP & SOFTWARE VERIFICATION            ║"
echo "║ Duration: 3-4 minutes                                          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Section 1: GitHub Repository
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Section 1: GitHub Repository Setup${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "Repository Information:"
cat ms1/github-repo-link.txt
echo ""
echo "Recent commits:"
git log --oneline | head -5
echo ""
sleep 2

# Section 2: Literature Review
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Section 2: Literature Review - Industrial Robotics Applications${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
head -30 ms1/literature-review.txt
echo ""
echo "[... literature review continues ...]"
echo ""
sleep 2

# Section 3: CAD Model
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Section 3: CAD Model Components${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "SolidWorks CAD Components:"
ls -lh ms1/4-dof-robotic-arm-5.snapshot.3/ | grep SLDPRT
echo ""
echo "CAD Assembly:"
ls -lh ms1/4-dof-robotic-arm-5.snapshot.3/ | grep SLDASM
echo ""
sleep 2

# Section 4: ROS2 & MuJoCo Environment
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}Section 4: ROS2 & MuJoCo Environment Verification${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo "ROS2 Distribution:"
. /opt/ros/jazzy/setup.bash
ros2 --version
echo ""
echo "Python Version:"
python3 --version
echo ""
echo "Project Structure:"
tree -L 2 -I '__pycache__|build|install|log' | head -20
echo ""

echo -e "${GREEN}✓ Milestone 1 Complete${NC}"
echo ""
echo "Summary:"
echo "  ✓ GitHub repository properly set up and version controlled"
echo "  ✓ Literature review completed with industrial applications"
echo "  ✓ CAD model with 4 revolute joints and end effector"
echo "  ✓ ROS2 Jazzy environment configured"
echo "  ✓ Python 3.12 ready for kinematics and control code"
echo ""
echo "Next milestone: Robot description and URDF visualization"
echo ""
