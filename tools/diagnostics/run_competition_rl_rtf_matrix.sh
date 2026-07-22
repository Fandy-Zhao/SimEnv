#!/usr/bin/env bash
set -euo pipefail

# Dry-run-first harness for the requested competition RL RTF-collapse matrix.
# It writes reproducible per-case metadata and command files. It only launches
# Gazebo when DRY_RUN=0 is explicitly set by the operator.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT_DIR="$REPO_ROOT/experiments/runs/0723_competition_rl_rtf_collapse"
AUTO_SH="$REPO_ROOT/auto.sh"
POLICY_PATH_DEFAULT="$REPO_ROOT/src/unitree_guide/logs/policy_act_inference_stair.pt"

DRY_RUN="${DRY_RUN:-1}"
WALL_TIMEOUT="${WALL_TIMEOUT:-900s}"

mkdir -p "$OUT_DIR"

declare -A MATRIX
MATRIX[M0]="WORLD_MODE=earth PHYSICS_PROFILE=normal GUI=false"
MATRIX[M1]="WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=false START_CONTROLLER=1 ENABLE_SENSOR_DATA=0 ENABLE_POINTCLOUD_CONVERTER=0 ENABLE_FAST_LIO2=0 ENABLE_RVIZ=0 ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0"
MATRIX[M2]="WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=false START_CONTROLLER=1 ENABLE_SENSOR_DATA=1 ENABLE_POINTCLOUD_CONVERTER=0 ENABLE_FAST_LIO2=0 ENABLE_RVIZ=0 ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0"
MATRIX[M3]="WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=false START_CONTROLLER=1 ENABLE_SENSOR_DATA=1 ENABLE_POINTCLOUD_CONVERTER=1 ENABLE_FAST_LIO2=0 ENABLE_RVIZ=0 ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0"
MATRIX[M4]="WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=false START_CONTROLLER=1 ENABLE_SENSOR_DATA=1 ENABLE_POINTCLOUD_CONVERTER=1 ENABLE_FAST_LIO2=1 ENABLE_RVIZ=0 ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0"
MATRIX[M5]="WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=false START_CONTROLLER=1 ENABLE_SENSOR_DATA=1 ENABLE_POINTCLOUD_CONVERTER=1 ENABLE_FAST_LIO2=1 ENABLE_RVIZ=0 ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0 RL_POLICY_PATH=$POLICY_PATH_DEFAULT"
MATRIX[M6]="WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=false START_CONTROLLER=1 ENABLE_SENSOR_DATA=0 ENABLE_POINTCLOUD_CONVERTER=0 ENABLE_FAST_LIO2=0 ENABLE_RVIZ=0 ENABLE_REFEREE_ODOM=0 ENABLE_GROUND_TRUTH=0 START_BUILDING_CONTROL=0 RL_POLICY_PATH=$POLICY_PATH_DEFAULT"
MATRIX[M7]="WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=false START_CONTROLLER=1 ENABLE_SENSOR_DATA=1 ENABLE_POINTCLOUD_CONVERTER=1 ENABLE_FAST_LIO2=1 ENABLE_RVIZ=0 ENABLE_REFEREE_ODOM=1 ENABLE_GROUND_TRUTH=1 START_BUILDING_CONTROL=1 RL_POLICY_PATH=$POLICY_PATH_DEFAULT"
MATRIX[M8]="WORLD_MODE=competition PHYSICS_PROFILE=normal GUI=false START_CONTROLLER=1 ENABLE_SENSOR_DATA=1 ENABLE_POINTCLOUD_CONVERTER=1 ENABLE_FAST_LIO2=1 ENABLE_RVIZ=0 ENABLE_REFEREE_ODOM=1 ENABLE_GROUND_TRUTH=1 START_BUILDING_CONTROL=1 RL_POLICY_PATH=$POLICY_PATH_DEFAULT OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1"

write_matrix_readme() {
  cat > "$OUT_DIR/README.md" <<'EOF'
# Competition RL RTF Collapse Matrix

This directory stores reproducible metadata and runtime samples for the M0-M8
controlled-variable diagnosis. Keep simulation time and wall time separate in
all analysis.

Use `DRY_RUN=1` first. Set `DRY_RUN=0` only when you are ready for this harness
to invoke `auto.sh`.
EOF
}

write_run_metadata() {
  local run_id="$1"
  local env_vars="$2"
  local run_dir="$OUT_DIR/$run_id"
  mkdir -p "$run_dir"

  cat > "$run_dir/env.txt" <<EOF
run_id=$run_id
timestamp=$(date -Iseconds)
git_head=$(git -C "$REPO_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)
git_branch=$(git -C "$REPO_ROOT" branch --show-current 2>/dev/null || echo unknown)
hostname=$(hostname)
dry_run=$DRY_RUN
wall_timeout=$WALL_TIMEOUT
env_vars=$env_vars
EOF

  {
    printf '%s\n' '#!/usr/bin/env bash'
    printf '%s\n' 'set -euo pipefail'
    printf '%s\n' 'REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"'
    for kv in $env_vars; do
      printf 'export %s\n' "$kv"
    done
    printf '%s\n' 'cd "$REPO_ROOT"'
    printf '%s\n' 'bash "$REPO_ROOT/auto.sh"'
  } > "$run_dir/command.sh"
  chmod 700 "$run_dir/command.sh"

  cat > "$run_dir/README.md" <<EOF
# $run_id

\`\`\`bash
cd "$REPO_ROOT"
$env_vars bash "$AUTO_SH"
\`\`\`

Runtime samplers can write JSON next to this file.
EOF
}

run_case() {
  local run_id="$1"
  local run_dir="$OUT_DIR/$run_id"
  echo "[launch] $run_id - starting auto.sh with timeout $WALL_TIMEOUT"
  timeout --foreground "$WALL_TIMEOUT" bash "$run_dir/command.sh"
}

write_matrix_readme

echo "=== Competition RL RTF Collapse Matrix ==="
echo "Repo: $REPO_ROOT"
echo "Out: $OUT_DIR"
echo "DRY_RUN: $DRY_RUN"
echo "Wall timeout per case: $WALL_TIMEOUT"
echo ""

for run_id in M0 M1 M2 M3 M4 M5 M6 M7 M8; do
  env_vars="${MATRIX[$run_id]}"
  write_run_metadata "$run_id" "$env_vars"
  if [ "$DRY_RUN" = "0" ]; then
    run_case "$run_id"
  else
    echo "[dry-run] $run_id - would run: $env_vars bash $AUTO_SH"
  fi
done

echo ""
echo "Matrix metadata written. Set DRY_RUN=0 to execute."
