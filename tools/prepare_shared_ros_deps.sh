#!/usr/bin/env bash
set -euo pipefail

# ---------------------------------------------------------------------------
# prepare_shared_ros_deps.sh
#
# Low-risk dependency-prep script.  It validates that pre-existing shared
# ROS source checkouts are at their expected commits and wires them into
# this repo via symlinks under src/external/.  It never clones, pulls,
# checkouts, patches, or modifies the shared directories.
# ---------------------------------------------------------------------------

# --- expected commits -------------------------------------------------------
FAST_LIO_EXPECTED="7cc4175de6f8ba2edf34bab02a42195b141027e9"
IKD_TREE_EXPECTED="e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4"
LIVOX_EXPECTED="3d240d5666129e1a3052e78ee8487a04b08fdda3"

# --- paths ------------------------------------------------------------------
REPO_ROOT="$(git rev-parse --show-toplevel)"
FAST_LIO_SRC="/home/zzf/search_ws/FAST_LIO"
LIVOX_SRC="/home/zzf/search_ws/livox_ros_driver"
EXTERNAL_DIR="${REPO_ROOT}/src/external"

# --- helper: check a shared source directory --------------------------------
check_shared_dir() {
  local dir="$1" expected="$2" label="$3"

  echo "=== Checking ${label} ==="

  # existence
  if [[ ! -d "${dir}" ]]; then
    echo "ERROR: ${label} directory not found at ${dir}" >&2
    exit 1
  fi
  echo "  path: ${dir}"

  # git repo
  if ! git -C "${dir}" rev-parse --git-dir >/dev/null 2>&1; then
    echo "ERROR: ${label} is not a git repository" >&2
    exit 1
  fi

  # clean working tree, including untracked files
  local status
  status="$(git -C "${dir}" status --short)"
  if [[ -n "${status}" ]]; then
    echo "ERROR: ${label} has uncommitted changes or untracked files" >&2
    echo "${status}" >&2
    exit 1
  fi
  echo "  working tree: clean"

  # at expected commit
  local actual
  actual="$(git -C "${dir}" rev-parse HEAD)"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "ERROR: ${label} HEAD is ${actual}, expected ${expected}" >&2
    exit 1
  fi
  echo "  HEAD: ${actual}"
}

# --- helper: verify symlink is git-ignored ----------------------------------
verify_git_ignore() {
  local path="$1"
  if git -C "${REPO_ROOT}" check-ignore -q "${path}" 2>/dev/null; then
    echo "  git check-ignore ${path}: OK (ignored)"
  else
    echo "ERROR: git check-ignore failed for ${path}: not covered by .gitignore" >&2
    exit 1
  fi
}

remove_legacy_link() {
  local path="$1"
  if [[ -L "${path}" ]]; then
    rm "${path}"
    echo "  removed legacy symlink: ${path}"
  elif [[ -e "${path}" ]]; then
    echo "ERROR: refusing to replace non-symlink legacy path: ${path}" >&2
    exit 1
  fi
}

# --- main -------------------------------------------------------------------

echo "Repo root: ${REPO_ROOT}"
echo ""

# 1. Validate shared source directories (read-only checks only)
check_shared_dir "${FAST_LIO_SRC}" "${FAST_LIO_EXPECTED}" "FAST_LIO"
echo ""

check_shared_dir "${LIVOX_SRC}" "${LIVOX_EXPECTED}" "livox_ros_driver"
echo ""

# 2. Check FAST_LIO submodule (include/ikd-Tree)
echo "=== Checking FAST_LIO submodule: include/ikd-Tree ==="
IKD_TREE_DIR="${FAST_LIO_SRC}/include/ikd-Tree"
if [[ ! -d "${IKD_TREE_DIR}" ]]; then
  echo "ERROR: ikd-Tree submodule directory not found at ${IKD_TREE_DIR}" >&2
  exit 1
fi
IKD_ACTUAL="$(git -C "${IKD_TREE_DIR}" rev-parse HEAD)"
if [[ "${IKD_ACTUAL}" != "${IKD_TREE_EXPECTED}" ]]; then
  echo "ERROR: ikd-Tree HEAD is ${IKD_ACTUAL}, expected ${IKD_TREE_EXPECTED}" >&2
  exit 1
fi
echo "  path: ${IKD_TREE_DIR}"
echo "  HEAD: ${IKD_ACTUAL}"
echo ""

# 3. Create src/external/ symlinks
echo "=== Creating symlinks under src/external/ ==="
mkdir -p "${EXTERNAL_DIR}"

remove_legacy_link "${REPO_ROOT}/src/FAST_LIO"
remove_legacy_link "${REPO_ROOT}/src/livox_ros_driver"

ln -sfn "${FAST_LIO_SRC}" "${EXTERNAL_DIR}/FAST_LIO"
echo "  ${EXTERNAL_DIR}/FAST_LIO -> ${FAST_LIO_SRC}"

ln -sfn "${LIVOX_SRC}" "${EXTERNAL_DIR}/livox_ros_driver"
echo "  ${EXTERNAL_DIR}/livox_ros_driver -> ${LIVOX_SRC}"
echo ""

# 4. Verify package.xml files exist (find maxdepth 2)
echo "=== Verifying package.xml presence ==="
FAST_LIO_PKGS="$(find -L "${EXTERNAL_DIR}/FAST_LIO" -maxdepth 2 -name package.xml | sort)"
LIVOX_PKGS="$(find -L "${EXTERNAL_DIR}/livox_ros_driver" -maxdepth 2 -name package.xml | sort)"

if [[ -z "${FAST_LIO_PKGS}" ]]; then
  echo "ERROR: no package.xml found in FAST_LIO (maxdepth 2)" >&2
  exit 1
fi
echo "  FAST_LIO package.xml:"
echo "${FAST_LIO_PKGS}" | while read -r f; do echo "    ${f}"; done

if [[ -z "${LIVOX_PKGS}" ]]; then
  echo "ERROR: no package.xml found in livox_ros_driver (maxdepth 2)" >&2
  exit 1
fi
echo "  livox_ros_driver package.xml:"
echo "${LIVOX_PKGS}" | while read -r f; do echo "    ${f}"; done
echo ""

# 5. Verify both symlinks are git-ignored
echo "=== Verifying git ignore coverage ==="
verify_git_ignore "src/external/FAST_LIO"
verify_git_ignore "src/external/livox_ros_driver"
echo ""

# 6. Summary
echo "=== Summary ==="
echo "FAST_LIO:        $(readlink -f "${EXTERNAL_DIR}/FAST_LIO")"
echo "  commit:         $(git -C "${FAST_LIO_SRC}" rev-parse HEAD)"
echo "  ikd-Tree:       $(git -C "${IKD_TREE_DIR}" rev-parse HEAD)"
echo "livox_ros_driver: $(readlink -f "${EXTERNAL_DIR}/livox_ros_driver")"
echo "  commit:         $(git -C "${LIVOX_SRC}" rev-parse HEAD)"
echo ""
echo "Done."
