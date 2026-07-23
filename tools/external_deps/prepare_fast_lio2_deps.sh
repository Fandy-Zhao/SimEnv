#!/usr/bin/env bash
set -euo pipefail

FAST_LIO_SOURCE="${FAST_LIO_SOURCE:-/home/zzf/search_ws/FAST_LIO}"
LIVOX_SOURCE="${LIVOX_SOURCE:-/home/zzf/search_ws/livox_ros_driver}"
FAST_LIO_SHA="7cc4175de6f8ba2edf34bab02a42195b141027e9"
IKD_TREE_SHA="e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4"
LIVOX_SHA="3d240d5666129e1a3052e78ee8487a04b08fdda3"
STAGING_ROOT="${SIMENV_FAST_LIO2_STAGING_ROOT:-/tmp/simenv-fast-lio2-deps/${FAST_LIO_SHA}-${LIVOX_SHA}}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
PATCH_DIR="$SCRIPT_DIR/patches"

usage() {
  cat <<'USAGE'
Usage:
  prepare_fast_lio2_deps.sh --check
  prepare_fast_lio2_deps.sh --prepare
  prepare_fast_lio2_deps.sh --clean-links
USAGE
}

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

require_clean_git() {
  local path="$1"
  local name="$2"
  local expected="$3"
  [[ -d "$path/.git" ]] || fail "$name source is not a Git repository: $path"
  local head
  head="$(git -C "$path" rev-parse HEAD)"
  [[ "$head" == "$expected" ]] || fail "$name HEAD mismatch: expected $expected, got $head"
  local status
  status="$(git -C "$path" status --short)"
  [[ -z "$status" ]] || fail "$name source is not clean: $path"$'\n'"$status"
}

check_sources() {
  [[ -d "$FAST_LIO_SOURCE" ]] || fail "FAST_LIO source not found: $FAST_LIO_SOURCE"
  [[ -d "$LIVOX_SOURCE" ]] || fail "livox_ros_driver source not found: $LIVOX_SOURCE"
  require_clean_git "$FAST_LIO_SOURCE" FAST_LIO "$FAST_LIO_SHA"
  require_clean_git "$LIVOX_SOURCE" livox_ros_driver "$LIVOX_SHA"

  local ikd_status ikd_head
  ikd_status="$(git -C "$FAST_LIO_SOURCE" submodule status --recursive include/ikd-Tree | awk '{print $1}')"
  ikd_head="${ikd_status#[-+ ]}"
  [[ "$ikd_head" == "$IKD_TREE_SHA" ]] || fail "ikd-Tree SHA mismatch: expected $IKD_TREE_SHA, got ${ikd_head:-MISSING}"

  [[ -f "$PATCH_DIR/fast_lio-cxx17.patch" ]] || fail "missing FAST_LIO patch"
  [[ -f "$PATCH_DIR/livox-driver-message-only.patch" ]] || fail "missing livox patch"

  echo "FAST_LIO_SOURCE=$FAST_LIO_SOURCE"
  echo "FAST_LIO_SHA=$FAST_LIO_SHA"
  echo "IKD_TREE_SHA=$IKD_TREE_SHA"
  echo "LIVOX_SOURCE=$LIVOX_SOURCE"
  echo "LIVOX_SHA=$LIVOX_SHA"
  echo "STAGING_ROOT=$STAGING_ROOT"
}

copy_source() {
  local source="$1"
  local dest="$2"
  mkdir -p "$(dirname "$dest")"
  rm -rf "$dest"
  rsync -a \
    --exclude='.git' \
    --exclude='build' \
    --exclude='devel' \
    --exclude='logs' \
    --exclude='Livox-SDK' \
    --exclude='*.bag' \
    --exclude='*.pcd' \
    "$source/" "$dest/"
}

apply_patch_file() {
  local dest="$1"
  local patch_file="$2"
  patch -d "$dest" -p1 < "$patch_file"
}

ensure_excluded() {
  local exclude_file
  exclude_file="$(git -C "$REPO_ROOT" rev-parse --git-path info/exclude)"
  mkdir -p "$(dirname "$exclude_file")"
  touch "$exclude_file"
  for entry in "/src/FAST_LIO" "/src/livox_ros_driver"; do
    if ! grep -Fxq "$entry" "$exclude_file"; then
      printf '%s\n' "$entry" >> "$exclude_file"
    fi
  done
}

create_links() {
  mkdir -p "$REPO_ROOT/src"
  for link in "$REPO_ROOT/src/FAST_LIO" "$REPO_ROOT/src/livox_ros_driver"; do
    if [[ -e "$link" && ! -L "$link" ]]; then
      fail "refusing to replace non-symlink path: $link"
    fi
  done
  ln -sfn "$STAGING_ROOT/FAST_LIO" "$REPO_ROOT/src/FAST_LIO"
  ln -sfn "$STAGING_ROOT/livox_ros_driver/livox_ros_driver" "$REPO_ROOT/src/livox_ros_driver"
  ensure_excluded
}

prepare() {
  check_sources
  copy_source "$FAST_LIO_SOURCE" "$STAGING_ROOT/FAST_LIO"
  copy_source "$LIVOX_SOURCE" "$STAGING_ROOT/livox_ros_driver"
  apply_patch_file "$STAGING_ROOT/FAST_LIO" "$PATCH_DIR/fast_lio-cxx17.patch"
  apply_patch_file "$STAGING_ROOT/livox_ros_driver" "$PATCH_DIR/livox-driver-message-only.patch"
  create_links
  echo "Prepared FAST-LIO2 staging packages."
  readlink -f "$REPO_ROOT/src/FAST_LIO"
  readlink -f "$REPO_ROOT/src/livox_ros_driver"
}

clean_links() {
  for link in "$REPO_ROOT/src/FAST_LIO" "$REPO_ROOT/src/livox_ros_driver"; do
    if [[ -L "$link" ]]; then
      rm "$link"
      echo "Removed $link"
    elif [[ -e "$link" ]]; then
      fail "refusing to remove non-symlink path: $link"
    fi
  done
}

case "${1:-}" in
  --check)
    check_sources
    ;;
  --prepare)
    prepare
    ;;
  --clean-links)
    clean_links
    ;;
  -h|--help)
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
