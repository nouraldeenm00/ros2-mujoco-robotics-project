#!/bin/bash
# Quick test script to verify all demonstrations work

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║  4-DOF ROBOT ARM - COMPLETE PROJECT VERIFICATION            ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

cd ~/ros2-mujoco-robotics-project

# Test 1: Console demonstrations
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "TEST 1: Console Demonstrations"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Running visual_demo.py..."
python3 ms3/demos/visual_demo.py

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Console demonstrations PASSED"
else
    echo ""
    echo "❌ Console demonstrations FAILED"
    exit 1
fi

# Test 2: Validation suite
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "TEST 2: Kinematics Validation Suite"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Running comprehensive validation tests..."
python3 ms3/demos/validate_kinematics.py

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Validation tests PASSED"
else
    echo ""
    echo "❌ Validation tests FAILED"
    exit 1
fi

# Summary
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                    ✅ ALL TESTS PASSED ✅                     ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "✓ Console demonstrations working"
echo "✓ All kinematics validated"
echo "✓ Pick and place operations verified"
echo "✓ Multi-part assembly verified"
echo ""
echo "Next Steps:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "To see LIVE 3D visualization in RViz2:"
echo ""
echo "  Terminal 1:"
echo "  $ cd ~/ros2-mujoco-robotics-project/ms2/ros2_ws"
echo "  $ source install/setup.bash"
echo "  $ ros2 launch robot_arm_description display.launch.py"
echo ""
echo "  Terminal 2 (after RViz2 loads):"
echo "  $ cd ~/ros2-mujoco-robotics-project/ms3/ros2_ws"
echo "  $ source install/setup.bash"
echo "  $ python3 src/robot_arm_kinematics/scripts/motion_visualizer.py"
echo ""
echo "  Watch the robot perform pick-and-place operations in 3D!"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📖 For complete documentation, see:"
echo "   - DEMO_GUIDE.md (this guide)"
echo "   - COMPLETE_DOCUMENTATION.md (technical details)"
echo "   - VISUALIZATION_GUIDE.md (visualization help)"
echo ""
