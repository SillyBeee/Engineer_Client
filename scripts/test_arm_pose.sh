#!/bin/bash
# 控制机械臂到指定关节位姿
# Run from project root: ./scripts/test_arm_pose.sh

set -e
DEMO="./build/demo_plan"

# 目标关节角度: {-1.53f, 0, 1.2f, 1.2f, 1.f, 1.f, 0.5f, 0.f, 0.0f}
$DEMO --plan-time 5 \
  -1.53 0 1.2 1.2 1.0 1.0 0.5 0.0 0.0
