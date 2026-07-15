#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /home/zzf/search_ws/SimEnv/devel/setup.bash

# Start controller
/home/zzf/search_ws/SimEnv/devel/lib/unitree_guide/junior_ctrl > /tmp/trot_test.log 2>&1 &
CTRL_PID=$!
echo "Controller PID=$CTRL_PID"

# Wait for feedback ready
for i in $(seq 1 20); do
    sleep 1
    if grep -q "feedback is ready" /tmp/trot_test.log 2>/dev/null; then
        echo "Feedback ready at ${i}s"
        break
    fi
done

# FixedStand
sleep 2
rostopic pub -1 /fsm/state_cmd std_msgs/Int8 "data: 2" 2>/dev/null
echo "Sent: 2 (FixedStand)"
sleep 5
grep -i "fixed stand" /tmp/trot_test.log && echo "✓ FixedStand entered" || echo "✗ FixedStand NOT entered"

# Trotting
rostopic pub -1 /fsm/state_cmd std_msgs/Int8 "data: 4" 2>/dev/null
echo "Sent: 4 (Trotting)"
sleep 5
grep -i "trotting" /tmp/trot_test.log && echo "✓ Trotting entered" || echo "✗ Trotting NOT entered"

# If Trotting entered, test movement
if grep -qi "trotting" /tmp/trot_test.log; then
    echo "=== Testing movement: 0.3 m/s forward ==="
    # Record start position
    rostopic echo -n 1 /ground_truth/base_w 2>/dev/null | grep "position:" -A2 > /tmp/trot_start.txt
    cat /tmp/trot_start.txt

    # Publish cmd_vel for 30s
    rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &
    CMD_PID=$!
    sleep 30
    kill $CMD_PID 2>/dev/null

    # Record end
    rostopic echo -n 1 /ground_truth/base_w 2>/dev/null | grep "position:" -A2 > /tmp/trot_end.txt
    cat /tmp/trot_end.txt
fi

echo "Done. Log: /tmp/trot_test.log"
