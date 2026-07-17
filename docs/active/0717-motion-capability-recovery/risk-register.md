# Risk Register: G1-RC Contact Readiness

| ID | Risk | Likelihood | Impact | Mitigation | Status |
|----|------|-----------|--------|------------|--------|
| R1 | `$SIMENV_BINARY_DEVEL/lib` missing `libunitreeFootContactPlugin.so` | HIGH | HIGH — no contact data at all | Build with `unitree_gazebo` in whitelist; verify plugin existence before trial | OPEN |
| R2 | Collision name mismatch in Gazebo SDF conversion | LOW | HIGH — no contact detection | The 0715 fix verified correct collision names; Gazebo version is pinned | MITIGATED |
| R3 | AsyncSpinner local variable lifetime (destructor stops spinner) | MEDIUM | LOW — callbacks dispatched by `spinOnce()` | `recvStateOnly()` calls `spinOnce()` each FSM iteration | MONITOR |
| R4 | Wall-clock staleness during very low RTF (<0.01) | LOW | MEDIUM — false contact loss abort | 1s wall-time window is generous; RTF below 0.01 is outside normal range | ACCEPTED |
| R5 | Robot spawn height causes feet to not contact ground initially | LOW | LOW — transient; robot settles within seconds | FixedStand holds until contact ready; height transition completes before readiness | ACCEPTED |
| R6 | `ENABLE_FOOT_FORCE_VISUAL=false` removes needed plugin | NONE | NONE — draw force plugin is visual only | Contact sensor is UNCONDITIONAL in gazebo.xacro | CLOSED |
| R7 | Catkin whitelist inconsistent between builds | MEDIUM | HIGH — plugins may be missing in some builds | Document minimum whitelist; add pre-flight plugin check | MITIGATED |
| R8 | Gazebo silently ignores plugin load failure | HIGH | MEDIUM — no error, no topic, hard to diagnose | New diagnostics track callback sequence; zero callbacks → plugin issue | MONITOR |
