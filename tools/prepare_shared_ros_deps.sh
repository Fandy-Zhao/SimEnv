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
LIVOX_SDK_URL="https://github.com/Livox-SDK/Livox-SDK.git"
LIVOX_SDK_REF="v2.3.0-8-g9306596"
LIVOX_SDK_EXPECTED="9306596a2bf15c1343bc023b497465ed0a32909d"

# --- paths ------------------------------------------------------------------
REPO_ROOT="$(git rev-parse --show-toplevel)"
FAST_LIO_SRC="/home/zzf/search_ws/FAST_LIO"
LIVOX_SRC="/home/zzf/search_ws/livox_ros_driver"
EXTERNAL_DIR="${REPO_ROOT}/src/external"
SHARED_DEPS_ROOT="/home/zzf/search_ws/shared_ros_deps"
LIVOX_SDK_ROOT="${SHARED_DEPS_ROOT}/Livox-SDK/${LIVOX_SDK_EXPECTED}"
LIVOX_SDK_SRC="${LIVOX_SDK_ROOT}/src"
LIVOX_SDK_BUILD="${LIVOX_SDK_ROOT}/build"
LIVOX_SDK_INSTALL="${LIVOX_SDK_ROOT}/install"
MODE="${1:---prepare}"

if [[ "${MODE}" != "--prepare" && "${MODE}" != "--check-only" ]]; then
  echo "Usage: tools/prepare_shared_ros_deps.sh [--prepare|--check-only]" >&2
  exit 2
fi

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

prepare_livox_sdk() {
  echo "=== Checking Livox-SDK external prefix ==="
  echo "  url: ${LIVOX_SDK_URL}"
  echo "  ref: ${LIVOX_SDK_REF}"
  echo "  commit: ${LIVOX_SDK_EXPECTED}"
  echo "  source: ${LIVOX_SDK_SRC}"
  echo "  build: ${LIVOX_SDK_BUILD}"
  echo "  install: ${LIVOX_SDK_INSTALL}"

  if [[ "${MODE}" == "--check-only" ]]; then
    [[ -d "${LIVOX_SDK_SRC}/.git" ]] || {
      echo "ERROR: Livox-SDK source checkout missing: ${LIVOX_SDK_SRC}" >&2
      exit 1
    }
    [[ -f "${LIVOX_SDK_INSTALL}/include/livox_sdk.h" ]] || {
      echo "ERROR: Livox-SDK header missing from install prefix" >&2
      exit 1
    }
    [[ -f "${LIVOX_SDK_INSTALL}/lib/liblivox_sdk_static.a" ]] || {
      echo "ERROR: Livox-SDK static library missing from install prefix" >&2
      exit 1
    }
    local check_actual check_status
    check_actual="$(git -C "${LIVOX_SDK_SRC}" rev-parse HEAD)"
    if [[ "${check_actual}" != "${LIVOX_SDK_EXPECTED}" ]]; then
      echo "ERROR: Livox-SDK HEAD is ${check_actual}, expected ${LIVOX_SDK_EXPECTED}" >&2
      exit 1
    fi
    check_status="$(git -C "${LIVOX_SDK_SRC}" status --short)"
    if [[ -n "${check_status}" ]]; then
      echo "ERROR: Livox-SDK source has uncommitted changes or untracked files" >&2
      echo "${check_status}" >&2
      exit 1
    fi
    return
  fi

  mkdir -p "${LIVOX_SDK_ROOT}"
  if [[ ! -d "${LIVOX_SDK_SRC}/.git" ]]; then
    if [[ -e "${LIVOX_SDK_SRC}" ]]; then
      echo "ERROR: Livox-SDK source path exists but is not a Git checkout: ${LIVOX_SDK_SRC}" >&2
      exit 1
    fi
    git clone "${LIVOX_SDK_URL}" "${LIVOX_SDK_SRC}"
  fi

  local remote_url status actual
  remote_url="$(git -C "${LIVOX_SDK_SRC}" remote get-url origin)"
  if [[ "${remote_url}" != "${LIVOX_SDK_URL}" ]]; then
    echo "ERROR: Livox-SDK remote is ${remote_url}, expected ${LIVOX_SDK_URL}" >&2
    exit 1
  fi

  git -C "${LIVOX_SDK_SRC}" fetch --tags origin
  git -C "${LIVOX_SDK_SRC}" checkout --detach "${LIVOX_SDK_EXPECTED}"

  actual="$(git -C "${LIVOX_SDK_SRC}" rev-parse HEAD)"
  if [[ "${actual}" != "${LIVOX_SDK_EXPECTED}" ]]; then
    echo "ERROR: Livox-SDK HEAD is ${actual}, expected ${LIVOX_SDK_EXPECTED}" >&2
    exit 1
  fi

  status="$(git -C "${LIVOX_SDK_SRC}" status --short)"
  if [[ -n "${status}" ]]; then
    echo "ERROR: Livox-SDK source has uncommitted changes or untracked files" >&2
    echo "${status}" >&2
    exit 1
  fi

  if [[ ! -f "${LIVOX_SDK_INSTALL}/include/livox_sdk.h" || ! -f "${LIVOX_SDK_INSTALL}/lib/liblivox_sdk_static.a" ]]; then
    mkdir -p "${LIVOX_SDK_BUILD}" "${LIVOX_SDK_INSTALL}"
    cmake -S "${LIVOX_SDK_SRC}" -B "${LIVOX_SDK_BUILD}" \
      -DCMAKE_BUILD_TYPE=Release \
      "-DCMAKE_CXX_FLAGS=-include memory" \
      -DCMAKE_INSTALL_PREFIX="${LIVOX_SDK_INSTALL}"
    cmake --build "${LIVOX_SDK_BUILD}" --target install -- -j"$(nproc)"
  fi

  [[ -f "${LIVOX_SDK_INSTALL}/include/livox_sdk.h" ]] || {
    echo "ERROR: Livox-SDK header missing from install prefix" >&2
    exit 1
  }
  [[ -f "${LIVOX_SDK_INSTALL}/lib/liblivox_sdk_static.a" ]] || {
    echo "ERROR: Livox-SDK static library missing from install prefix" >&2
    exit 1
  }
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

# 6. Prepare independent Livox-SDK prefix
prepare_livox_sdk
echo ""

# 7. Summary
echo "=== Summary ==="
echo "FAST_LIO:        $(readlink -f "${EXTERNAL_DIR}/FAST_LIO")"
echo "  commit:         $(git -C "${FAST_LIO_SRC}" rev-parse HEAD)"
echo "  ikd-Tree:       $(git -C "${IKD_TREE_DIR}" rev-parse HEAD)"
echo "livox_ros_driver: $(readlink -f "${EXTERNAL_DIR}/livox_ros_driver")"
echo "  commit:         $(git -C "${LIVOX_SRC}" rev-parse HEAD)"
echo "Livox-SDK:       ${LIVOX_SDK_INSTALL}"
echo "  source:         ${LIVOX_SDK_SRC}"
echo "  commit:         $(git -C "${LIVOX_SDK_SRC}" rev-parse HEAD)"
echo ""
echo "Done."
