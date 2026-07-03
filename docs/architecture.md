# Architecture

## Overview

SimEnv 是一个 ROS1 Noetic + Gazebo Classic 仿真工作区。系统围绕**多楼层室内建筑生成 → 仿真启动 → 机器人控制 → 危险源评估**这条数据流组织。

## System Diagram

```
generate_competition_scene.py
    ├── building_generator_core (Python lib)
    │       ├── constraints.py      → BuildingConstraints
    │       ├── layout.py           → BuildingLayout, FloorLayout, RoomSpec
    │       ├── generator.py        → randomized building assembly
    │       └── exporter.py         → SDF/XML world generation
    ├── building_generator_classic (Python lib)
    │       ├── classic_export.py   → door_config.yaml, elevator_config.yaml
    │       ├── control_server.py   → ROS service server for door/elevator
    │       └── control_runtime.py  → state machines
    └── outputs:
        ├── generated_building/competition_scene.world
        ├── generated_building/door_config.yaml
        ├── generated_building/elevator_config.yaml
        └── results/danger_truth.json
                │
                ▼
auto.sh  ──►  roslaunch unitree_guide multi_floor_gazeboSim.launch
                ├── Gazebo Classic + competition_scene.world
                ├── Unitree A1 model (unitree_guide)
                ├── Livox Mid-360 LiDAR (Mid360_imu_sim)
                ├── RealSense depth camera
                └── building_generator_classic_control (door/elevator services)
                │
                ▼
junior_ctrl (RL controller)  ◄── /cmd_vel (geometry_msgs/Twist)
                │
                ▼
Competition algorithm  ──►  results/detected_danger.json
                │
                ▼
evaluate_danger.py  ──►  results/evaluation_result.json
```

## Package Dependency Graph

```
building_obstacles
    ├── building_generator_core
    └── building_generator_classic
            └── building_generator_interfaces (ROS msgs/srvs)

unitree_guide          (独立; 依赖 Gazebo + ROS controller)
Mid360_imu_sim          (独立; Gazebo plugin)
uav_simulator/          (独立; 与地面机器人解耦)
    ├── local_sensing
    ├── mockamap
    ├── so3_quadrotor_simulator
    ├── so3_control
    └── map_generator
```

## Key Interfaces

| Interface | Direction | Protocol |
|-----------|-----------|----------|
| `/cmd_vel` | Contest algorithm → Robot | `geometry_msgs/Twist` |
| `/Odometry_gazebo` | Gazebo → Contest algorithm | `nav_msgs/Odometry` |
| `/scan` | Livox Mid-360 → Contest algorithm | `sensor_msgs/PointCloud2` |
| `/camera/image_raw` | RGB camera → Contest algorithm | `sensor_msgs/Image` |
| `/real_sense/depth/points` | Depth camera → Contest algorithm | `sensor_msgs/PointCloud2` |
| Door/Elevator control | building_generator_classic ↔ Gazebo | ROS services + Gazebo model state |

## Data Flow

1. **Scene Generation** (`generate_competition_scene.py`): randomized building layout → SDF world + metadata + ground truth
2. **Simulation Launch** (`auto.sh`): world loading → Gazebo + sensors → robot controller
3. **Control** (`junior_ctrl`): receives `/cmd_vel`, applies RL locomotion
4. **Detection** (contest algorithm): reads sensor data, detects danger sources
5. **Evaluation** (`evaluate_danger.py`): compares `detected_danger.json` against `danger_truth.json`

## Tech Stack

- **OS**: Ubuntu 20.04
- **ROS**: Noetic (ros-comm, catkin)
- **Simulator**: Gazebo Classic 11
- **Robot**: Unitree A1 (unitree_guide stack)
- **Python**: 3.8+ (building generation + evaluation)
- **C++**: C++14 (Gazebo plugins, UAV sim, controller)
- **Build**: CMake via catkin_make
