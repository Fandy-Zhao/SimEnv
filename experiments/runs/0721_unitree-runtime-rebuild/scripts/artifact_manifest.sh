#!/usr/bin/env bash
set -euo pipefail

OUTDIR="${1:?Usage: $0 <output-directory>}"
DEVEL_SETUP="${2:-devel/setup.bash}"

if [ ! -f "$DEVEL_SETUP" ]; then
  echo "ERROR: $DEVEL_SETUP not found" >&2
  exit 1
fi

source "$DEVEL_SETUP" || { echo "ERROR: failed to source $DEVEL_SETUP" >&2; exit 1; }

mkdir -p "$OUTDIR"
MANIFEST="$OUTDIR/artifact_manifest.log"
exec > >(tee -i "$MANIFEST") 2>&1

echo "=== Artifact Manifest ==="
echo "Timestamp: $(date -Iseconds)"
echo "Setup:     $DEVEL_SETUP"

echo "=== ROS_PACKAGE_PATH ==="
echo "${ROS_PACKAGE_PATH:-<unset>}"

echo "=== CMAKE_PREFIX_PATH ==="
echo "${CMAKE_PREFIX_PATH:-<unset>}"

echo "=== rospack list ==="
rospack list 2>&1 || echo "rospack failed"
for pkg in unitree_guide unitree_gazebo unitree_legged_control unitree_legged_msgs; do
  echo "rospack find $pkg: $(rospack find "$pkg" 2>/dev/null || echo NOT_FOUND)"
done

DEVEL_DIR="$(cd "$(dirname "$DEVEL_SETUP")" 2>/dev/null && pwd || dirname "$DEVEL_SETUP")"
echo "=== Devel Artifacts (${DEVEL_DIR}) ==="
find "$DEVEL_DIR" -maxdepth 1 -mindepth 1 | sort | while read -r entry; do
  echo "  $(basename "$entry") ($(du -sh "$entry" 2>/dev/null | cut -f1 || echo '?'))"
done

echo "=== SHA256 Checksums ==="
mapfile -t sha256_files < <(
  find "$DEVEL_DIR" -type f \( \
    -name 'junior_ctrl' -o \
    -name 'state_from_gazebo' -o \
    -name '*.so' -o \
    -name '*.so.*' \
  \) -print 2>/dev/null | sort
)
for extra in \
  src/unitree_guide/logs/policy_act_inference_plane.pt \
  src/unitree_guide/logs/policy_act_inference_stair.pt \
  src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world; do
  [ -f "$extra" ] && sha256_files+=("$extra")
done

if [ ${#sha256_files[@]} -gt 0 ]; then
  for f in "${sha256_files[@]}"; do
    sha256sum "$f"
  done | sort -k2 > "$OUTDIR/hashes.txt"
  echo "Hashes written to $OUTDIR/hashes.txt ($(wc -l < "$OUTDIR/hashes.txt") files)"
else
  echo "No hash-able artifacts found"
fi

echo "=== readlink / ldd key artifacts ==="
for f in "${sha256_files[@]}"; do
  echo "--- $f ---"
  readlink -f "$f" 2>/dev/null || true
  case "$f" in
    *.so|*.so.*|*/junior_ctrl|*/state_from_gazebo)
      ldd "$f" 2>&1 || true
      ;;
  esac
done

echo "=== Manifest Complete ==="
