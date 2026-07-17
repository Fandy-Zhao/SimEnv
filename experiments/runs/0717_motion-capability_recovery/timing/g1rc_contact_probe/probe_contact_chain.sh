#!/usr/bin/env bash
# G1-RC Contact Chain Runtime Probe
# Usage: source this script after Gazebo is running, or run with:
#   bash probe_contact_chain.sh > probe_output.txt 2>&1
set -euo pipefail

OUTDIR="${1:-/tmp/g1rc_contact_probe}"
mkdir -p "$OUTDIR"

echo "=== G1-RC Contact Chain Probe ==="
echo "Timestamp: $(date -Iseconds)"
echo "Output: $OUTDIR"
echo ""

# ---- C0: Physical contact ----
echo "--- C0: Gazebo Model State ---"
if rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; then
  rostopic echo -n 1 /gazebo/model_states 2>/dev/null | tee "$OUTDIR/c0_model_states.txt" || echo "C0: model_states not available"
else
  echo "C0: /gazebo/model_states topic not found"
fi

echo ""
echo "--- C0: Gazebo Link States ---"
if rostopic list 2>/dev/null | grep -q "/gazebo/link_states"; then
  rostopic echo -n 1 /gazebo/link_states 2>/dev/null | tee "$OUTDIR/c0_link_states.txt" || echo "C0: link_states not available"
else
  echo "C0: /gazebo/link_states topic not found"
fi

# ---- C1: Plugin loading ----
echo ""
echo "--- C1: Gazebo Plugin Load Check ---"
PLUGIN_LOG="/tmp/gz_plugin_check.log"
if [ -f "$HOME/.gazebo/gzserver.log" ]; then
  grep -iE "plugin|contact|error|failed|cannot|symbol|library" "$HOME/.gazebo/gzserver.log" 2>/dev/null | tail -30 | tee "$OUTDIR/c1_plugin_log.txt" || echo "C1: No plugin errors in log"
elif [ -f "/tmp/gzserver.log" ]; then
  grep -iE "plugin|contact|error|failed|cannot|symbol|library" "/tmp/gzserver.log" 2>/dev/null | tail -30 | tee "$OUTDIR/c1_plugin_log.txt" || echo "C1: No plugin errors in log"
else
  echo "C1: No gzserver log found at standard locations"
fi

# ---- C2/C3: Contact topics ----
echo ""
echo "--- C3: ROS Topic List ---"
rostopic list 2>/dev/null | tee "$OUTDIR/c3_topic_list.txt" || echo "C3: rostopic list failed (ROS master not running?)"

echo ""
echo "--- C3: Contact Force Topics ---"
for leg in FR FL RR RL; do
  TOPIC="/visual/${leg}_foot_contact/the_force"
  echo ""
  echo "Checking $TOPIC ..."
  if rostopic list 2>/dev/null | grep -qF "$TOPIC"; then
    echo "  Topic EXISTS: $TOPIC"
    # Topic type
    TYPE=$(rostopic type "$TOPIC" 2>/dev/null || echo "unknown")
    echo "  Type: $TYPE"
    # Topic info
    rostopic info "$TOPIC" 2>/dev/null | tee "$OUTDIR/c3_${leg}_info.txt" || echo "  info failed"
    # Topic rate (5s sample)
    echo "  Rate (5s sample):"
    timeout 6 rostopic hz "$TOPIC" 2>/dev/null | tail -5 | tee "$OUTDIR/c3_${leg}_hz.txt" || echo "  hz failed"
    # Last 3 messages
    echo "  Last 3 messages:"
    rostopic echo -n 3 "$TOPIC" 2>/dev/null | tee "$OUTDIR/c3_${leg}_echo.txt" || echo "  echo failed"
  else
    echo "  Topic MISSING: $TOPIC"
  fi
done

# ---- C4: ROS node list ----
echo ""
echo "--- C4: ROS Nodes ---"
rosnode list 2>/dev/null | tee "$OUTDIR/c4_node_list.txt" || echo "C4: rosnode list failed"

echo ""
echo "--- C4: ROS Node Info ---"
for node in $(rosnode list 2>/dev/null || true); do
  echo "Node: $node"
  rosnode info "$node" 2>/dev/null | head -20 | tee -a "$OUTDIR/c4_node_info.txt" || true
  echo ""
done

# ---- Plugin binary check ----
echo ""
echo "--- Plugin Binary Check ---"
WS_DEVEL="${SIMENV_BINARY_DEVEL:-}"
if [ -z "$WS_DEVEL" ]; then
  WS_DEVEL="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)/devel"
fi
echo "Devel path: $WS_DEVEL"
for plugin in libunitreeFootContactPlugin.so libunitreeDrawForcePlugin.so; do
  PLUGIN_PATH="$WS_DEVEL/lib/$plugin"
  if [ -f "$PLUGIN_PATH" ]; then
    echo "  EXISTS: $PLUGIN_PATH ($(stat -c%s "$PLUGIN_PATH") bytes)"
    ldd "$PLUGIN_PATH" 2>/dev/null | grep -E "not found|=>" | tee "$OUTDIR/c1_ldd_${plugin}.txt" || true
  else
    echo "  MISSING: $PLUGIN_PATH"
  fi
done

# ---- GAZEBO_PLUGIN_PATH ----
echo ""
echo "--- Environment ---"
echo "GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH:-unset}"
echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-unset}"
echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-unset}"
echo "robot_name=$(rosparam get /robot_name 2>/dev/null || echo 'unset')"
echo "use_sim_time=$(rosparam get /use_sim_time 2>/dev/null || echo 'unset')"

echo ""
echo "=== Probe Complete ==="
echo "Results saved to: $OUTDIR"
ls -la "$OUTDIR/"
