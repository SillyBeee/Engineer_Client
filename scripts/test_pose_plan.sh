#!/bin/bash
# Test PlanToPose (end-effector pose) with various targets
# Run from project root: ./scripts/test_pose_plan.sh

set -e
DEMO="./build/demo_plan"

echo "=== Test 1: Forward reach (x positive) ==="
$DEMO --pose --plan-time 5 0.40 0.0 0.45 0.0 0.0 0.0

echo "=== Test 2: Side reach (y positive) ==="
$DEMO --pose --plan-time 5 0.20 0.35 0.45 0.0 0.0 0.0

echo "=== Test 3: High reach with pitch ==="
$DEMO --pose --plan-time 5 0.30 0.0 0.60 0.0 -0.5 0.0

echo "=== Test 4: Repeating trajectory ==="
$DEMO --repeat --pose --plan-time 5 0.35 0.0 0.40 0.0 0.0 0.0

echo "=== All pose plan tests complete ==="
