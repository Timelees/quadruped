#!/bin/bash
# Automated test runner for obstacle optimization
set -e
WS=/home/lee/Code/quadruped_control_ws
source $WS/install/setup.bash
export ROS_DOMAIN_ID=${1:-71}
TEST_NAME=${2:-v1}
LOG_DIR=$WS/log/obstacle_opt_${TEST_NAME}
mkdir -p $LOG_DIR

# Cleanup
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f ruby.*gz 2>/dev/null || true
pkill -9 -x rviz2 2>/dev/null || true
sleep 2

# Launch gazebo
echo "[runner] launching gazebo..."
ros2 launch ocs2_quadruped_controller gazebo.launch.py \
    world:=stair10cm_obst8 x:=-2.5 y:=0.0 z:=0.5 yaw:=0.0 \
    > $LOG_DIR/gazebo.log 2>&1 &
GAZEBO_PID=$!

# Wait for odom topic
echo "[runner] waiting for odom (timeout 60s)..."
for i in $(seq 1 60); do
    if ros2 topic list 2>/dev/null | grep -q "/odom"; then
        echo "[runner] odom ready after ${i}s"
        break
    fi
    sleep 1
done

# Additional wait for controller
sleep 5

# Run test
echo "[runner] starting test..."
python3 $WS/src/quadruped_ros2_control/test/src/full_course_test.py \
    > $LOG_DIR/test.log 2>&1 &
TEST_PID=$!

# Wait for test to complete
wait $TEST_PID || true

# Cleanup
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f ruby.*gz 2>/dev/null || true
pkill -9 -x rviz2 2>/dev/null || true

echo "==================== TEST LOG ===================="
tail -50 $LOG_DIR/test.log
echo "==================================================="
