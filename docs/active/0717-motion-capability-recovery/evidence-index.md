# Evidence Index: G1-RC Contact Readiness

## Static Evidence (This Investigation)

| ID | Path | Description | Status |
|----|------|-------------|--------|
| S1 | `src/unitree_guide/unitree_ros/unitree_gazebo/plugin/foot_contact_plugin.cc` | Contact sensor plugin source | Reviewed |
| S2 | `src/unitree_guide/unitree_ros/robots/a1_description/xacro/gazebo.xacro` | Contact sensor URDF definition | Reviewed |
| S3 | `src/unitree_guide/unitree_ros/robots/a1_description/xacro/leg.xacro` | Leg kinematic chain (single fixed foot joint) | Reviewed |
| S4 | `src/unitree_guide/unitree_guide/unitree_guide/src/interface/IOROS.cpp` | IOROS contact force consumption | Reviewed + Enhanced |
| S5 | `src/unitree_guide/unitree_guide/unitree_guide/include/message/LowlevelState.h` | LowlevelState footForce/footForceValid | Reviewed + Enhanced |
| S6 | `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_Trotting.cpp` | Trotting readiness gate | Reviewed + Enhanced |
| S7 | `src/unitree_guide/unitree_guide/unitree_guide/include/control/CtrlComponents.h` | Control components (contact = VecInt4*) | Reviewed |
| S8 | `src/unitree_guide/unitree_guide/unitree_guide/src/Gait/WaveGenerator.cpp` | Gait phase and contact calculation | Reviewed |
| S9 | `devel/lib/libunitreeFootContactPlugin.so` | Built plugin binary (348KB) | Verified |
| S10 | `devel/lib/libunitreeDrawForcePlugin.so` | Built draw plugin binary (386KB) | Verified |

## Runtime Evidence (Pending Execution)

| ID | Path | Description | Status |
|----|------|-------------|--------|
| R1 | `g1rc_contact_probe/c0_model_states.txt` | Gazebo model pose at FixedStand | PENDING |
| R2 | `g1rc_contact_probe/c1_plugin_log.txt` | Gazebo plugin load messages | PENDING |
| R3 | `g1rc_contact_probe/c3_*_echo.txt` | Per-foot WrenchStamped samples | PENDING |
| R4 | `g1rc_contact_probe/c3_*_hz.txt` | Per-foot topic rates | PENDING |
| R5 | `g1rc_trot_2ms_low_rtf/timing.csv` | FSM timing records (2ms, low RTF) | PENDING |
| R6 | `g1rc_trot_2ms_low_rtf/contact.csv` | Per-foot force time series | PENDING |
| R7 | `g1rc_trot_2ms_low_rtf/trial_metrics.json` | Aggregated metrics | PENDING |
| R8 | `g1rc_trot_2ms_low_rtf/trial_status.json` | Pass/fail verdict | PENDING |
| R9–R18 | `g1rc_trot_2ms_high_rtf/` through `g1rc_trot_4ms/` | Remaining matrix trials | PENDING |

## Previous Evidence (0715 Trotting Safety)

| ID | Path | Description | Status |
|----|------|-------------|--------|
| P1 | `experiments/runs/0715_trotting-safety/notes.md` | Contact fix verification: forces [10.9, 11.1, 12.6, 12.9] N | Verified |
| P2 | `experiments/runs/0715_trotting-safety/issue.md` | Issue: IOROS did not consume contact force; leg.xacro duplicate joint | Fixed |

## Git Commits (This Branch)

| Commit | Description |
|--------|-------------|
| `4a820a34` | test(g1rc): add contact chain callback diagnostics |
