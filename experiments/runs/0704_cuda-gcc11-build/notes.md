# Experiment Notes: 0704 cuda gcc11 build

## Compiler Status

| Item | Result |
|------|--------|
| default gcc | 12.3.0 — cc1plus: `cc1plus` (bare, can't find) |
| default g++ | 11.4.0 — cc1plus: `/usr/lib/gcc/.../11/cc1plus` |
| gcc-11 path | `/usr/bin/gcc-11` (11.4.0) |
| g++-11 path | `/usr/bin/g++-11` (11.4.0) |
| gcc-11 cc1plus | `/usr/lib/gcc/.../11/cc1plus` ✅ |
| g++-11 cc1plus | `/usr/lib/gcc/.../11/cc1plus` ✅ |
| selected CC | `/usr/bin/gcc-11` |
| selected CXX | `/usr/bin/g++-11` |
| CUDAHOSTCXX | `/usr/bin/g++-11` |

## CUDA / Torch Status

| Item | Result |
|------|--------|
| CUDA_HOME | `/usr/local/cuda-11.8` |
| nvcc | `/usr/local/cuda-11.8/bin/nvcc` |
| nvcc + g++-11 smoke test | ✅ PASS (compiled + ran successfully) |
| torch version | 2.0.1+cu118 |
| torch cuda available | True |
| torch cmake prefix | `.venv/lib/.../torch/share/cmake` |
| Torch_DIR | `.venv/lib/.../torch/share/cmake/Torch` |

## Build Script Changes

`tools/build_with_venv.sh` now:
- Auto-selects gcc-11/g++-11 when available (cc1plus check)
- Accepts SIMENV_CC/SIMENV_CXX overrides
- Detects CUDA_HOME and adds nvcc to PATH
- Sets CC/CXX/CUDAHOSTCXX environment variables
- Passes CMAKE_C_COMPILER/CXX_COMPILER/CUDA_HOST_COMPILER to cmake
- Passes CUDA_TOOLKIT_ROOT_DIR/CUDAToolkit_ROOT
- Accumulates ROS + Torch + CUDA into CMAKE_PREFIX_PATH
- Warns about CMakeCache compiler mismatch
- Does NOT auto-delete build/devel

## Build Results

| Test | Result | Detail |
|------|--------|--------|
| bash -n | ✅ PASS | |
| nvcc + g++-11 smoke | ✅ PASS | CUDA compile + run |
| build_with_venv.sh | ⚠️ CMakeCache mismatch | 33 compiler-change warnings, CMake needs clean build |
| CUDA host compiler errors | ✅ **FIXED** | 0 CUDA/gcc errors! |

## Remaining Blockers

| Category | Status |
|----------|--------|
| CUDA_HOST_COMPILER | ✅ FIXED |
| CMAKE_CACHE_COMPILER_MISMATCH | ⚠️ Needs `rm -rf build devel` |
| LIVOX_CXX11_STILL_FAILING | ❌ Livox SDK C++11 issues |
| TORCH_DEPENDENCY | ⚠️ kineto_LIBRARY-NOTFOUND (torch internal) |
| ROS_DEPENDENCY | ⚠️ move_base_msgs not found |

## Next Step

User must confirm: `rm -rf build devel && ./tools/build_with_venv.sh`
