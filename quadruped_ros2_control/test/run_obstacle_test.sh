#!/bin/bash
# Run obstacle/stair traversal test.
# Usage: bash run_obstacle_test.sh [test_name]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"

TEST_NAME="${1:-obst_test_$(date +%Y%m%d_%H%M%S)}"
LOG_DIR="$WS_DIR/log/obstacle_opt_$(date +%Y-%m-%d)/$TEST_NAME"
mkdir -p "$LOG_DIR"
echo "[run_obstacle_test] Test: $TEST_NAME"
echo "[run_obstacle_test] Logs: $LOG_DIR"

# Source workspace
source "$WS_DIR/install/setup.bash"

# Kill any leftover gz/rviz
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f ruby.*gz 2>/dev/null || true
pkill -9 -x rviz2 2>/dev/null || true
sleep 1

export ROS_DOMAIN_ID=$((RANDOM % 90 + 10))
echo "[run_obstacle_test] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# Launch gazebo in background
ros2 launch ocs2_quadruped_controller gazebo.launch.py \
    world:=stair10cm_obst8 \
    x:=0.0 y:=0.0 z:=0.5 yaw:=0.0 \
    > "$LOG_DIR/gazebo.log" 2>&1 &
GAZEBO_PID=$!
echo "[run_obstacle_test] gazebo PID=$GAZEBO_PID"

# Wait for gazebo to be ready
echo "[run_obstacle_test] Waiting for Gazebo to start (20s)..."
sleep 20

# Start the obstacle test node in background
python3 "$SCRIPT_DIR/../test/src/obstacle_test.py" \
    > "$LOG_DIR/obstacle_test.log" 2>&1 &
TEST_PID=$!
echo "[run_obstacle_test] test node PID=$TEST_PID"

# Wait for test to finish (with timeout)
TIMEOUT=120
START_TIME=$(date +%s)
while kill -0 $TEST_PID 2>/dev/null; do
    sleep 2
    NOW=$(date +%s)
    ELAPSED=$((NOW - START_TIME))
    if [ $ELAPSED -gt $TIMEOUT ]; then
        echo "[run_obstacle_test] TIMEOUT after ${TIMEOUT}s"
        break
    fi
done

# Cleanup
echo "[run_obstacle_test] Test finished. Cleaning up..."
kill $TEST_PID 2>/dev/null || true
kill $GAZEBO_PID 2>/dev/null || true
sleep 2
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f ruby.*gz 2>/dev/null || true
pkill -9 -x rviz2 2>/dev/null || true

echo "[run_obstacle_test] Done. Logs in $LOG_DIR"
echo "==================== OBSTACLE TEST LOG ===================="
tail -50 "$LOG_DIR/obstacle_test.log" 2>/dev/null || true
