#!/bin/bash
# Test PlanToTarget (joint angles) with various targets
# Run from project root: ./scripts/test_joint_plan.sh

set -e
DEMO="./build/demo_plan"

echo "=== Test 1: Small joint motion (radians) ==="
$DEMO --plan-time 3 0.5 -0.3 0.8 -0.2 0.6 -0.4 0.3 -0.1 0.2

echo "=== Test 2: Large joint motion (radians) ==="
$DEMO --plan-time 5 1.2 -0.8 2.0 -0.6 1.5 -1.0 1.0 -0.5 0.8

echo "=== Test 3: Joint motion in degrees ==="
$DEMO --degrees --plan-time 3 30 -20 45 -15 35 -25 20 -10 15

echo "=== Test 4: Repeating trajectory ==="
$DEMO --repeat --plan-time 3 0.8 -0.4 1.2 -0.3 0.9 -0.6 0.5 -0.2 0.4

echo "=== All joint plan tests complete ==="
