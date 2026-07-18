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
