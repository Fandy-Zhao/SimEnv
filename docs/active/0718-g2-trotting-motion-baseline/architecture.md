# G2-B Architecture

## Data Flow
```
Gazebo sim-time
  ├── Ground truth pose/twist (50 Hz) ──→ G2 metrics recorder
  ├── Controller state topic (500 Hz) ──→ validity checker
  └── Contact states (500 Hz) ──→ contact duty monitor

Trial runner (auto.sh / Python)
  ├── Spawn → FSM chain → Command → Record → Stop
  ├── Validity gate per trial
  └── Epoch aggregation per speed
```

## No Controller/Physics/Model Edits
G2-B is a baseline measurement phase. The controller, physics parameters, and URDF/SDF model are left unchanged from the G1_R_PASS state. Any deviation requires a documented ADR and explicit branch fork.

## Component Isolation
- Trial runner: orchestrates sim lifecycle, emits commands, records data.
- Validity checker: post-hoc per-trial gate (contact, torque, RTF, FSM state).
- Metrics computer: aggregates valid trials per speed into epoch-median metrics.
- Reporter: emits baseline report template, root-cause placeholders.

## Frame Convention
Gazebo world frame (ENU: x-forward, y-left, z-up). Estimator output (odom frame) may differ; discrepancy is a documented risk (see risk register). Metrics are computed in Gazebo world frame.
