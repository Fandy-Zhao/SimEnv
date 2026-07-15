# Experiment Notes: 0704 fast-lio2-workspace-docs

## Goal

Fix FAST-LIO2 integration documentation to clarify that SimEnv itself is the catkin workspace root and FAST_LIO should be at `SimEnv/src/FAST_LIO`.

## Changes

- Replaced `cd <catkin_ws>/src` with `cd SimEnv/src` in all docs
- Clarified "外部 catkin workspace 依赖" → "FAST_LIO 放在 SimEnv/src/ 下"
- Updated config comments to reference `tools/build_with_venv.sh`
- No code changes; documentation only

## Verification

- `grep` confirmed no remaining `<catkin_ws>` references outside `src/FAST_LIO/` (upstream package)
- All docs now explicitly state SimEnv is the workspace root
