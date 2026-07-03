# AGENTS.md

## Project Goal

ROS1 Noetic + Gazebo Classic 仿真比赛环境。模拟多楼层室内建筑中的 Unitree A1 机器狗，用于完成未知环境探索和危险源识别任务。环境启动时随机生成多楼层楼栋、危险源（红色球体）、干扰源（红色方块/绿色球体）、门、电梯和传感器链路。

比赛目标：控制机器狗完成室内探索，识别并输出危险源位置。

## Hard Rules

1. Do not modify unrelated files.
2. Do not create temporary files in the repository root.
3. Put temporary analysis, logs, and experiment records under `experiments/runs/MMDD_short-name/`.
4. Every meaningful code change must update at least one status document: `PROJECT_STATE.md`, `CHANGELOG.md`, `docs/module_status.md`, or a relevant module README.
5. Do not directly delete deprecated files. Move them to `docs/deprecated/MMDD-short-name/` and add a `README.md` explaining why they were deprecated, what replaces them, and whether they can later be deleted.
6. Before edits, report `git status --short`, current branch, files read, and the plan.
7. After edits, report `git status --short`, `git diff --stat`, and relevant tests or smoke checks.
8. Final reports must include changed files, reasons, tests, risks, and next steps.

## Workflow

Issue -> Branch -> Plan -> Diff -> Commit -> Report

## Branch Policy

- `feat/MMDD-short-name`
- `fix/MMDD-short-name`
- `docs/MMDD-short-name`
- `chore/MMDD-short-name`
- `exp/MMDD-short-name`
- `refactor/MMDD-short-name`

Do not develop directly on `master` unless the task is trivial or the user explicitly requests it.

## Commit Policy

- `feat(scope): short description`
- `fix(scope): short description`
- `docs(scope): short description`
- `chore(scope): short description`
- `refactor(scope): short description`
- `test(scope): short description`
- `exp(scope): short description`

Avoid vague messages such as `update`, `fix bug`, `change files`, `final`, or `temp`.

## Documentation Policy

- `PROJECT_STATE.md`: current project status, active branch plan, known risks.
- `ROADMAP.md`: planned milestones and backlog.
- `CHANGELOG.md`: date-ordered user-visible and engineering changes.
- `docs/architecture.md`: system architecture and major dependencies.
- `docs/module_status.md`: module ownership, status, and validation notes.
- `docs/decisions/ADR-MMDD-short-name.md`: architectural or technical decisions.
- `docs/reports/MMDD_short-name.md`: task reports, audits, or integration summaries.
- `experiments/runs/MMDD_short-name/notes.md`: experiment notes, command logs, and results.

## Deprecated File Policy

Move deprecated files to `docs/deprecated/MMDD-short-name/`. Include a `README.md` with original location, deprecation reason, replacement, and deletion guidance. Ask before deleting files or archived directories.

## Test Policy

- **Build check**: `catkin_make -j` in workspace root. Must pass before committing non-trivial changes.
- **Python syntax**: `python3 -m py_compile` on changed `.py` files. Prefer `python3 -m pytest` for packages with test directories.
- **Python packages with `test/`**: `building_generator_core` and `building_generator_classic` use `nosetests` via catkin. Run via `catkin_make run_tests` or directly with `python3 -m pytest`.
- **C++ modules**: compile check via `catkin_make`. Focus smoke testing on launch-file startup and topic publishing.
- **Runtime smoke test**: after scene-generation changes, run `python3 setup_multi_floor_simulation.py` to verify scene generation completes. For control/logic changes, launch `auto.sh` with `START_CONTROLLER=0` to validate Gazebo + sensor startup.
- **Evaluation regression**: when modifying `building_obstacles/scripts/evaluate_danger.py`, run the evaluation command from the README against a known truth/detected pair.
- If a check cannot be run (e.g. no Gazebo display available), report the reason and residual risk.

## Project-Specific Modules

| Module | Path | Purpose |
|--------|------|---------|
| `building_obstacles` | `src/building_obstacles/` | Competition scene generation, danger source spawning, and evaluation scripts |
| `building_generator_core` | `src/building_generator_core/` | Python core library: building layout, constraint solving, random generation |
| `building_generator_classic` | `src/building_generator_classic/` | Classic Gazebo export pipeline and door/elevator control runtime |
| `building_generator_interfaces` | `src/building_generator_interfaces/` | ROS message/service definitions for building control |
| `unitree_guide` | `src/unitree_guide/` | Unitree A1 controller (`junior_ctrl`), RL locomotion, joystick interface |
| `Mid360_imu_sim` | `src/Mid360_imu_sim/` | Livox Mid-360 LiDAR simulation plugin and IMU platform |
| `uav_simulator` | `src/uav_simulator/` | UAV simulation sub-packages: local sensing, mockamap, SO3 quadrotor, map generator |
