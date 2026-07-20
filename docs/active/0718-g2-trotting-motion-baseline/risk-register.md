# G2-B Risk Register

| ID | Risk | Source | Likelihood | Impact | Mitigation |
|----|------|--------|------------|--------|------------|
| R01 | G1 4 ms effective gait not completed | G1 inheritance | High | Medium | Accept; document as G2-B scope boundary |
| R02 | G1 pause/reset extended matrix not completed | G1 inheritance | Medium | Low | Flag if pause/reset is needed; use fresh spawn per trial |
| R03 | RL timing not part of G2 | G1 inheritance | High | Medium | G2-B measures motion tracking only; RL policy is frozen |
| R04 | Low RTF may extend wall-clock runtime | Runtime | Medium | Medium | Set RTF ≥ 0.5 flag; do not reject trials on RTF alone |
| R05 | State/contact topic frequency may constrain measurement | Data quality | Medium | Medium | Record callback freshness and sample counts; mark affected trials invalid or inconclusive |
| R06 | Short windows may hide steady-state behavior | Methodology | Medium | Medium | Use the defined last-3s steady window; extend only to 5 epochs for high variance, not infinite retries |
| R07 | Trial-to-trial spawn/contact variation | Simulation | High | Medium | 3 epochs per speed; report median not mean |
| R08 | Estimator and Gazebo frame mismatch | Data quality | Medium | High | Compute metrics in Gazebo world frame; document estimator offset separately |
| R09 | Contact loss may invalidate trials | Simulation | Medium | High | Require fresh four-foot contact readiness before WAVE_ALL and preserve invalid evidence with reason codes |
| R10 | 原 fall validator 可能将正常 frame 基准误判为翻倒 | Validator semantics | High | High | Gate V audits pose source, quaternion/frame semantics, and validates with FixedStand/runtime evidence before changing behavior |
| R11 | 修复 validator 可能改变 trial validity，但不能改变控制行为 | Validation | Medium | High | Only edit validator/reclassification semantics; preserve locomotion code and compare accepted control sequence in smoke revalidation |
| R12 | 离线重判可能受旧数据字段不完整限制 | Evidence quality | Medium | Medium | Report missing base/trunk/link fields explicitly and do not overclaim runtime posture from incomplete CSVs |
| R13 | WAVE_ALL 阻塞可能独立于 fall validator | Root cause | High | High | Preserve `WAVE_ALL_NOT_REACHED` and `GAIT_NOT_ADVANCING`; defer readiness/start/cancel root cause to Gate P |
| R14 | 旧 non-finite 现象可能不可稳定复现 | Runtime | Medium | High | Do not authorize G2-R from stale NaN alone; require Gate P first-block evidence or repeatable non-finite capture |
| R15 | 诊断日志可能影响 RTF | Runtime overhead | Medium | Medium | Keep Gate V/P probes minimal and record RTF/diagnostic scope in reports |
| R16 | readiness 和 wave cancel 原因目前缺乏结构化字段 | Observability | High | High | Gate P adds first-block taxonomy and structured readiness/start/cancel fields without changing control behavior |
| R17 | P0 FixedStand 共享底座未建立，导致 Trotting/RL active 归因不安全 | Runtime | High | High | Gate A stopped after P0 failure; P1/P2 and active RL are blocked until four-foot contact and FixedStand entry are recovered |
