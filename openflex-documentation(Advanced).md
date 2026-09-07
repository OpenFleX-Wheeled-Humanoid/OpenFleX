# OpenFlex Wheeled Humanoid Robot User Manual (Advanced)

English | [中文](./openflex-说明文档(高级)-CN.md)

---

> This document covers the perception, mapping, localization, navigation, VLA data collection/training/inference, and core algorithm details for the OpenFlex robot system.
>
> For hardware control, VR teleoperation, and subsystem basics, refer to the Basic manual.

---

## Table of Contents

- [Chapter 1: Sensor System](#chapter-1-sensor-system)
- [Chapter 2: Mapping](#chapter-2-mapping)
- [Chapter 3: Localization](#chapter-3-localization)
- [Chapter 4: Navigation](#chapter-4-navigation)
- [Chapter 5: Whole-Body VLA Data Collection, Training and Inference](#chapter-5-whole-body-vla-data-collection-training-and-inference)
- [Appendix: Core Algorithm Details and References](#appendix-core-algorithm-details-and-references)

---

## Chapter 1: Sensor System

### 1.1 Quick Start

The primary sensor is the Livox Mid-360 3D LiDAR with built-in IMU. Network configuration is required before starting the driver.

| Item | Value |
|------|-------|
| Host IP | 192.168.1.50 |
| Netmask | 255.255.255.0 |
| MID360 IP | 192.168.1.173 |

```bash
sudo ip addr add 192.168.1.50/24 dev eno1
sudo ip link set eno1 up
ping 192.168.1.173

ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 topic hz /livox/lidar   # ~10 Hz
ros2 topic hz /livox/imu     # ~200 Hz
```

### 1.2 Livox Mid-360 Specifications

| Parameter | Value |
|-----------|-------|
| Range | 0.3~40 m |
| FOV | 360° × 59° |
| Point rate | ~200,000 pts/s |
| Built-in IMU | 6-axis, 200 Hz |
| Interface | Ethernet |
| ROS message | `livox_ros_driver2/msg/CustomMsg` |

**Config:** `src/openflex_chassis/hardware_sensor_layer/livox_ros_driver2/config/MID360_config.json`

### 1.3 Other Sensors

- Intel D435: optional (enable via `use_camera:=true`)
- RPLiDAR C1: backup 2D LiDAR (URDF placeholder)

---

## Chapter 2: Mapping

### 2.1 Quick Start

```bash
export MAP_NAME=your_map
mkdir -p "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME"
ros2 launch swerve_navigation mapping.launch.py

# Save map
ros2 service call /pgo/save_maps interface/srv/SaveMaps \
  "{file_path: '$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME', save_patches: true}"

# Convert 3D→2D
mkdir -p "$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME"
ros2 run pgo pcd_to_nav2_map \
  --pcd "$HOME/MID_360_nav/openflex_maps/3d/$MAP_NAME/map.pcd" \
  --out-dir "$HOME/MID_360_nav/openflex_maps/2d/$MAP_NAME" \
  --resolution 0.05 --obstacle-z-min 0.10 --obstacle-z-max 1.40
```

### 2.2 FAST-LIO2 Algorithm

Tightly-coupled LiDAR-Inertial Odometry using IESKF + ikd-Tree.

**Code:** `src/openflex_chassis/mapping_localization_layer/fastlio2/src/lio_node.cpp`

Per-frame pipeline: IMU pre-integration → point undistortion → IESKF update (point-to-plane) → ikd-Tree map update.

| Parameter | Mapping | Navigation |
|-----------|---------|-----------|
| `world_frame` | map | odom |
| `body_frame` | mid360_link | base_link |
| `ieskf_max_iter` | 4 | 3 |
| `publish_map_odom` | true | false |

### 2.3 PGO

Loop closure via ScanContext + ICP. Publishes `/pgo/offset` correction. Key params: `loop_search_radius=1.0m`, `loop_score_tresh=0.15`.

**Code:** `src/openflex_chassis/mapping_localization_layer/pgo/`

### 2.4 3D→2D Map Conversion

Z-slice projection → SOR filtering → raycasting → inflation. Output: `map.yaml` + `map.pgm` for Nav2.

---

## Chapter 3: Localization

### 3.1 Architecture

```
ICP: live cloud + PCD map → map→odom
safe_odom_mux: LIO + wheel odom → odom→base_link (50 Hz)
Nav2 uses: map→base_link = map→odom × odom→base_link
```

### 3.2 ICP Three-Level Strategy

**Code:** `src/openflex_chassis/mapping_localization_layer/icp_registration/`

1. ScanContext → NDT → ICP (~3-5s)
2. ScanContext → ICP directly (NDT fails)
3. Grid search ICP (~10-15s, last resort)

Key params: `thresh=0.5`, `xy_offset=5.0m`, `yaw_resolution=30°`, `continuous_realign_interval=5.0s`.

### 3.3 Continuous Realignment

Every 5s: skip ScanContext, use current map→odom as initial guess, run ICP refinement. Non-blocking (separate thread). Failure preserves last valid transform.

### 3.4 Navigation-Mode TF Tree

| Transform | Provider | Rate |
|-----------|----------|------|
| map→odom | icp_registration | ~100 Hz |
| odom→base_link | safe_odom_mux | 50 Hz |
| base_link→sensors | robot_state_publisher | static |

---

## Chapter 4: Navigation

### 4.1 Quick Start

```bash
ros2 launch swerve_navigation full_system.launch.py \
  pcd_path:=/path/to/map.pcd \
  map_yaml:=/path/to/map.yaml \
  initial_pose:="[0,0,0,0,0,0]"
```

### 4.2 Nav2 Stack

**Config:** `src/openflex_chassis/navigation_layer/swerve_navigation/config/nav2_params.yaml`

| Component | Plugin |
|-----------|--------|
| Controller | `nmpc_controller::NmpcController` |
| Planner | `nav2_smac_planner/SmacPlanner2D` |
| Smoother | SimpleSmoother |
| Behaviors | Spin, Backup, Wait |
| Velocity Smoother | VelocitySmoother |

### 4.3 NMPC Controller

**Code:** `src/openflex_chassis/navigation_layer/nmpc_controller/`

Model: `state=[px,py,θ,vx,vy,ω]`, `control=[ax,ay,α]`. acados SQP_RTI, N=20, Tf=2.0s.

| Limit | Value |
|-------|-------|
| vx_max | 0.50 m/s |
| vy_max | 0.25 m/s |
| omega_max | 1.0 rad/s |

Features: costmap cost penalty, local A* obstacle repair, rotate-to-heading, swerve-native lateral tracking.

### 4.4 Costmap

- Local: 5×5m rolling, `/scan` obstacle layer, inflation 0.30m
- Global: static layer + obstacle + inflation
- Footprint: 0.74×0.62m rectangle

### 4.5 Velocity Smoother

`max_velocity=[0.50, 0.22, 1.0]`, `max_accel=[0.8, 0.6, 1.5]`, open-loop feedback.

---

## Chapter 5: Whole-Body VLA Data Collection, Training and Inference

### 5.1 Overview

Uses LeRobot's Robot/Follower + Teleoperator/Leader abstraction. VR teleoperation provides real-time control; LeRobot records observations and actions. During inference, trained policy outputs actions back to ROS2.

Note: VLA data collection, training, and inference are optional extension modules. The current OpenFlex base source package does not include the `openflex_vla` implementation. Commands using `<openflex_vla_package>` must be replaced with the actual ROS package name provided by the installed VLA extension.

### 5.2 Quick Start

```bash
# Hardware + SLAM + cameras
ros2 launch <openflex_vla_package> wholebody_vla_record.launch.py

# VR teleoperation
ros2 launch openarmx_integrated_bringup integrated_vr_teleop.launch.py

# Record
python3 scripts/lerobot_record_openflex.py \
  --robot.type=openarmx_follower_ros2 \
  --robot.skip_send_action=true \
  --robot.ros2.enable_base=true \
  --robot.ros2.base_action_mode=pose \
  --dataset.repo_id=YOUR_NAME/task --dataset.fps=15
```

### 5.3 Schema

**Observation:** 16 arm joints + 2 head + 2 lift + 3 base twist + 3 episode-local pose + 4 RGB cameras = 26 scalars + 4 images.

**Action (common):** 16 arm + 2 head + 1 lift = 19 dims. Base action depends on mode:
- `pose`: +3 (base_target_x/y/theta) = 22 total
- `trajectory`: +24 (8 future points × 3) = 43 total  
- `twist`: +3 (vx/vy/wz) = 22 total

### 5.4 Training & Inference

Policies: ACT (50-200 eps), Diffusion Policy (100-500), VLA models (500+).

Inference: close VR, set `skip_send_action=false`. `pose/trajectory` modes publish `/vla/base_path`; `twist` publishes `/cmd_vel` directly.

---

## Appendix: Core Algorithm Details and References

### A.1 Swerve Kinematics

IK: `vx_i = vx - ω*yi`, `vy_i = vy + ω*xi`, `angle = atan2(vy_i, vx_i)`, `speed = sqrt(...)`.
FK: Least-squares pseudo-inverse. Midpoint integration for odometry.

Refs: Muir & Neuman 1987; Campion et al. 1996.

### A.2 FAST-LIO2

IESKF state: position, velocity, rotation, IMU biases, LiDAR-IMU extrinsics. Point-to-plane residuals in ikd-Tree.

Refs: Xu et al. "FAST-LIO2" T-RO 2022; Cai et al. "ikd-Tree" 2021.

### A.3 FAST-LIVO2

LiDAR + IMU + camera fusion. Visual constraints complement geometric features in degraded environments.

Refs: Zheng et al. "FAST-LIVO/FAST-LIVO2" 2022/2024.

### A.4 ScanContext + NDT + ICP

ScanContext: polar descriptor (20 rings × 60 sectors), RingKey KD-tree search. NDT: normal distribution grid matching. ICP: iterative closest point with point-to-plane.

Refs: Kim & Kim 2018/2021; Biber & Strasser 2003; Besl & McKay 1992.

### A.5 PGO / iSAM2

Pose graph with odometry + loop factors. Incremental Bayes tree optimization.

Refs: Kaess et al. "iSAM2" IJRR 2012; Dellaert "GTSAM" 2012.

### A.6 NMPC / acados

Nonlinear MPC, SQP_RTI single iteration per cycle, HPIPM QP solver. Costmap-aware stage cost.

Refs: Mayne et al. Automatica 2000; Verschueren et al. "acados" MPC 2022.

### A.7 VR IK & Safety

Pinocchio damped least-squares IK, step limiting, joint clamping. Head: relative quaternion control with soft limits.

Refs: Carpentier et al. "Pinocchio" 2019; Siciliano et al. 2009.

### A.8 LeRobot VLA

ACT action chunking, Diffusion Policy denoising, episode-local odom design avoiding global drift.

Refs: Zhao et al. "ACT" RSS 2023; Chi et al. "Diffusion Policy" RSS 2023; Cadene et al. "LeRobot" 2024.

### A.9 Full Reference List

| Domain | References |
|--------|-----------|
| Swerve kinematics | Muir & Neuman 1987; Campion et al. 1996 |
| LiDAR-Inertial | Xu et al. FAST-LIO2 2022; Cai et al. ikd-Tree 2021 |
| Visual-Inertial | Zheng et al. FAST-LIVO 2022; FAST-LIVO2 2024 |
| Place recognition | Kim & Kim ScanContext 2018/2021 |
| Registration | Besl & McKay ICP 1992; Biber & Strasser NDT 2003 |
| Graph SLAM | Kaess et al. iSAM2 2012; Dellaert GTSAM 2012 |
| Navigation | Hart et al. A* 1968; Macenski et al. Nav2 2020 |
| NMPC | Mayne et al. 2000; Verschueren et al. acados 2022 |
| Robotics/IK | Carpentier et al. Pinocchio 2019; Nakamura & Hanafusa 1986 |
| CANopen | CiA 301; CiA 402 |
| VLA | ACT 2023; Diffusion Policy 2023; RT-1/RT-2 2023; OpenVLA 2024; pi0 2024; LeRobot 2024 |

---

**End of Advanced Manual.**
