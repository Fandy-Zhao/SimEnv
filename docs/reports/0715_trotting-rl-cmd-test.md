# Task Report: Trotting and RL headless `/cmd_vel` validation

## Branch

- Baseline committed to `master`: `2a36abbe`.
- Task branch: `exp/0715-trotting-rl-cmd-validation`.
- No merge or remote push was performed.

## Summary

Neither locomotion state currently supports reliable programmatic movement:

| State | FSM entry | Zero command | `linear.x=+0.3` tracking | Result |
|---|---:|---|---|---|
| Trotting | pass | Gazebo/IMU velocity becomes NaN | unsafe to attempt | fail |
| RL | pass | finite, but drifts and turns | wrong magnitude/direction; turns | fail |

The actual velocity interface is `/cmd_vel`, not `/cmd`. State selection is
through `/fsm/state_cmd` (`4`=Trotting, `6`=RL).

## Test setup

The accepted test used a deterministic one-floor scene (`SEED=71501`), no GUI,
no RViz/FAST-LIO2/nonessential sensors, and default locomotion timing: Gazebo
2 ms / 500 Hz / ODE 40 and controller `dt=0.002`. The Torch-enabled controller
loaded `policy_act_inference_plane.pt` successfully.

An attempted 4 ms / 250 Hz / ODE 20 profile was rejected because FixedStand
became unstable. It is not a valid locomotion acceptance profile without
retuning.

## Runtime evidence

### Startup and baseline

`auto.sh` does not command FixedStand when FAST-LIO2 is disabled. Also,
`/fsm/state_cmd` is cleared before the full-feedback guard returns, so an early
one-shot command can be lost. Repeating `data: 2` around unpause produced a
reliable transition. At `/clock=15.302 s`, FixedStand was finite and nearly
stationary at approximately `(-0.01156, 2.29873, 0.12001)`.

### Trotting

A zero Twist was published before `data: 4`. The log confirmed
`fixed stand -> trotting`. The `a1_gazebo` model still existed and both
controller/Gazebo processes remained alive, but model velocity and IMU became
NaN before any nonzero command. The failure therefore originates inside or
immediately downstream of Trotting output, not topic connectivity.

### RL paired measurement

RL stayed finite, allowing an adjacent before/after pair:

| Sample | `/clock` | Position `(x,y,z)` m | Approx. yaw |
|---|---:|---|---:|
| before | 34.748 s | `(0.022906, 1.974482, 0.101569)` | +26.6° |
| after | 39.434 s | `(0.022591, 1.938075, 0.102628)` | +17.0° |

Across 4.686 simulated seconds with body-frame `linear.x=+0.3 m/s`, displacement
was `(-0.000315, -0.036406) m`, while yaw changed about `-9.6°`. At the initial
heading a valid response should have substantial world +X/+Y displacement.
The command is received but not tracked.

## Cause analysis

### Trotting: high-confidence fault path

`State_Trotting::_dYawCmdPast` is declared but never initialized in the
constructor or `enter()`. The first `getUserCmd()` immediately computes:

```text
_dYawCmd = 0.9 * _dYawCmdPast + 0.1 * requested_yaw_rate
```

An indeterminate value can enter `_yawCmd`, `_Rd`, balance-force calculation,
inverse kinematics, and Gazebo joint commands. This is the strongest match for
immediate NaN under a zero Twist. One instrumented run should still identify
the exact first non-finite value among `_dYawCmd`, `_dWbd`, balance forces,
`_qGoal`, `_qdGoal`, and `_tau`.

The fork also starts an AMP thread on every Trotting entry, hard-codes velocity
limits, and adds `/cmd_vel`; these should be disabled for the first minimal A/B.
The core gait/balance algorithm otherwise closely follows Unitree's code.

### RL: policy/model contract mismatch

The strongest static mismatch is joint ordering. Feedback and actions use
`reindex={3,4,5,0,1,2,9,10,11,6,7,8}`, while the default joint tensor's hip
sign pattern matches the current Gazebo FR/FL/RR/RL ordering. Reordering only
some parts of the observation/action contract can feed incorrect left/right
offsets to the policy and explains persistent yaw bias.

Additional material risks:

- no training config, ordered joint names, normalization metadata, control
  decimation, policy checksum, or exporter version for the `.pt` files;
- a 50 Hz inference worker writes `_lowCmd` concurrently with the 500 Hz main
  publisher without synchronization;
- the worker is created before its RUNNING flag is set;
- RL `/cmd_vel` has no finite/range validation or freshness timeout;
- this 225-to-12 history policy is project-specific, not official Unitree code.

## Should official `unitree_guide` be referenced?

Yes for Trotting, with care. The [official repository](https://github.com/unitreerobotics/unitree_guide)
expects FixedStand-to-Trotting walking, and its
[Trotting implementation](https://github.com/unitreerobotics/unitree_guide/blob/main/unitree_guide/src/FSM/State_Trotting.cpp)
is the right A/B baseline. However, it also leaves `_dYawCmdPast` uninitialized
and targets Ubuntu 18.04/ROS Melodic, so it should not be copied wholesale.

Official `unitree_guide` is insufficient for RL because it has no equivalent
RL state or policy. The required reference is the original training/export
repository that produced `policy_act_inference_plane.pt`.

## Files Changed

- `experiments/runs/0715_trotting-rl-cmd-test/issue.md`: governed scope.
- `experiments/runs/0715_trotting-rl-cmd-test/notes.md`: measurements/diagnosis.
- `experiments/runs/0715_trotting-rl-cmd-test/command.txt`: reproduction.
- `docs/reports/0715_trotting-rl-cmd-test.md`: detailed report.
- `docs/module_status.md`: current locomotion validation status.

No control code was changed on the task branch.

## Tests

- `catkin_make -j`: pass before baseline commit.
- Headless FixedStand under default physics: pass.
- Trotting zero-command state entry: fail, non-finite Gazebo/IMU.
- RL zero-command stability: fail (finite but drift/yaw bias).
- RL paired forward command: fail velocity tracking.
- Official comparison: commit
  `fdf4d23de6affe8ee38fb4d892f61053fa1fcbcb` (2024-05-07).

## Documentation Updated

This report, experiment Issue/notes/commands, and `docs/module_status.md`.

## Git

- Master baseline commit: `2a36abbe`.
- Task documentation commit: recorded after final checks.
- Nothing merged or pushed.

## Risks

- Trotting's first non-finite variable is inferred from code/runtime
  propagation and needs one instrumented run for proof.
- RL cannot be repaired safely without the original policy contract.
- The custom spawn/URDF, Noetic, and Gazebo setup differ from the official
  Melodic example.
- Early state-command loss can invalidate future tests unless orchestration is
  fixed.

## Next Step

1. Initialize all Trotting command state, add finite guards before motor output,
   disable AMP code, and run official core versus fork on the same world/model.
2. Recover RL training metadata and verify a named joint mapping plus one
   recorded observation/action frame offline.
3. Replace worker-thread `_lowCmd` writes with a synchronized action buffer and
   add `/cmd_vel` finite/range/freshness handling.
4. Only after zero-command stability passes, run forward/lateral/yaw steps with
   exact simulated-time windows and automatic safety stop.
