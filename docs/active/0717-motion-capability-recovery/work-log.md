# Work Log: 0717 Motion Capability Recovery

## 2026-07-18: G1-RC Contact Readiness Investigation

### Branch
`fix/0717-g1r-contact-readiness` (to be created from `integrate/0718-g1-fixed-sim-scheduler`)

### Context
G1-R validation at baseline `05d69a8a` returned `G1_R_FAIL` because Trotting never entered `WAVE_ALL`. The readiness log showed `contact=0 force=[0.0 0.0 0.0 0.0]N`, indicating the contact force data chain was broken.

### Static Analysis Findings

1. **Contact sensor plugins** (`libunitreeFootContactPlugin.so`): Always loaded unconditionally in `gazebo.xacro` (lines 86-121). Built by `unitree_gazebo/CMakeLists.txt`. Present in `devel/lib/`.

2. **Topic chain**: Gazebo contact sensor → `/visual/{FR,FL,RR,RL}_foot_contact/the_force` (WrenchStamped) → IOROS::updateFootForce() → `_foot_force[]` (atomic float) → recvState() → LowlevelState.footForce[] → State_Trotting readiness check.

3. **Previous fix (0715)**: The IOROS contact force consumption was added and verified working with forces ~[10.9, 11.1, 12.6, 12.9] N. The leg.xacro duplicate joint bug was fixed.

4. **Launch parity**: Isolated runner uses identical launch files as normal auto.sh. Differences: binary isolation via devel symlink, env vars (GUI=false, etc.).

5. **Most likely root cause**: The isolated runner's `$SIMENV_BINARY_DEVEL/lib` symlink target may not include `libunitreeFootContactPlugin.so`, causing Gazebo to silently skip loading the contact sensors.

### Action Plan
1. Add contact chain diagnostics to IOROS and State_Trotting
2. Rebuild with proper whitelist including `unitree_gazebo`
3. Verify plugins are in devel/lib
4. Run smoke test with runtime topic probe
5. Complete G1-R matrix
6. Publish report

### Files to modify
- `src/unitree_guide/unitree_guide/unitree_guide/src/interface/IOROS.cpp` (diagnostics)
- `src/unitree_guide/unitree_guide/unitree_guide/include/interface/IOROS.h` (diagnostics)
- `src/unitree_guide/unitree_guide/unitree_guide/src/FSM/State_Trotting.cpp` (enhanced readiness logging)
- `src/unitree_guide/unitree_guide/unitree_guide/include/message/LowlevelState.h` (contact age tracking)

### Build configuration
- `SIMENV_CATKIN_WHITELIST="unitree_legged_msgs;unitree_guide;unitree_legged_control;unitree_gazebo"`
- `CC=/usr/bin/gcc-11 CXX=/usr/bin/g++-11 CUDAHOSTCXX=/usr/bin/g++-11`
- `UNITREE_ENABLE_TORCH_POLICY=ON`
