# Experiment Notes: 0704 fast-lio2 build check

## Environment Snapshot (2026-07-04)

| Item | Status | Detail |
|------|--------|--------|
| Workspace root | ✅ | SimEnv is catkin workspace root |
| ROS Noetic | ✅ | /opt/ros/noetic/setup.bash exists |
| Python | ✅ | 3.10.12 (system), 3.10.12 (venv) |
| .venv | ✅ | exists, all Python deps importable |
| torch (Python) | ✅ | 2.0.1+cu118 in .venv |
| catkin_make | ✅ | /opt/ros/noetic/bin/catkin_make |
| rospack | ✅ | /opt/ros/noetic/bin/rospack |
| roslaunch | ✅ | /opt/ros/noetic/bin/roslaunch |
| FAST_LIO | ✅ | src/FAST_LIO, submodule ikd-Tree OK |
| simenv_fast_lio2_integration | ✅ | rospack find OK |
| PCL | ✅ | libpcl-dev installed |
| Eigen | ✅ | libeigen3-dev installed |
| ros-noetic-pcl-ros | ✅ | installed |
| ros-noetic-tf* | ✅ | all tf/tf2 packages installed |
| ros-noetic-cv-bridge | ✅ | installed |
| libtorch (C++ SDK) | ❌ | Missing — blocks catkin_make |

## Build Attempt

- Script: `tools/build_with_venv.sh`
- Python: `.venv/bin/python` (3.10.12, torch 2.0.1 available)
- ROS: Noetic (/opt/ros/noetic/setup.bash)
- Result: **FAILED** at CMake configuration stage
- Root cause: `unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt:13` requires libtorch at hardcoded path `/home/ros/Guoyulun/Download/libtorch`
- Classification: **ENV_MISSING** (pre-existing, not caused by FAST-LIO2 integration)

## Static Check Results

| Test | Result |
|------|--------|
| py_compile (system Python 3.10) | PASS |
| py_compile (venv Python 3.10) | PASS |
| rospack find simenv_fast_lio2_integration | PASS |
| rospack find fast_lio | PASS |
| roslaunch --files | PASS |
| check_repo_clean.py | PASS (no issues) |

## Error Classification

- **PASS**: py_compile, rospack, roslaunch, repo check
- **ENV_MISSING**: libtorch (C++ SDK) at hardcoded path `/home/ros/Guoyulun/Download/libtorch`
- **CODE_ISSUE**: unitree_guide hardcodes user-specific libtorch path — pre-existing debt, not in this task's scope
- **RUNTIME_BLOCKED**: Gazebo + X server smoke tests (not attempted)

## Recommended Fix (for user)

Install libtorch to match unitree_guide's expected path or update CMakeLists.txt:

```bash
# Option A: Install libtorch where unitree_guide expects it
mkdir -p /home/ros/Guoyulun/Download
cd /home/ros/Guoyulun/Download
wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcu118.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cu118.zip

# Option B: Create symlink on this machine
sudo mkdir -p /home/ros/Guoyulun/Download
sudo ln -s /actual/libtorch/path /home/ros/Guoyulun/Download/libtorch
```
