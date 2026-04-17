# nav2-tracer-agilex

Nav2 Humble autonomous navigation stack for **Tracer Agilex 2.0** + **Livox Mid-360** LiDAR.
Built for the **SocialTech Challenge** — hospital corridor navigation with dynamic obstacles.

**Platform:** Jetson Orin NX 16GB | ROS2 Humble

---

## Hardware

| Component | Spec |
|-----------|------|
| Robot base | Tracer Agilex 2.0 — differential drive, 702×610×169mm |
| LiDAR | Livox Mid-360 — 360° H / -7°+52° V, 200k pts/s @10Hz |
| IMU | Livox Mid-360 internal (tightly coupled with LiDAR) |
| Compute | Jetson Orin NX 16GB |

---

## Stack Overview

```
Livox Mid-360
├── /livox/lidar (PointCloud2)  ──→  FAST-LIO2  ──→  /odom_lidar  ──→  AMCL  ──→  /map→odom TF
│                                                                         ↑
│                               pc_to_laserscan  ──→  /scan  ────────────┘
│
├── Global costmap: static_layer + STVL (FOV-aware decay) + inflation (0.08m)
├── Local costmap:  obstacle_layer (2D) + voxel_layer (3D) + inflation
│
├── Planner:     SmacPlanner2D (cost-aware A*) + SimpleSmoother
├── Controller:  RotationShim → MPPI (2000 samples @30Hz, DiffDrive)
│
├── CollisionMonitor: ApproachZone (2s TTC) + SlowdownZone (0.3x) + StopZone
└── VelocitySmoother → /cmd_vel → Tracer CAN bus
```

**Map:** Pre-built offline with GLIM (fixed structure only — no gurneys/mobile objects).

---

## Package Structure

```
nav2_tracer_agilex/
├── config/
│   ├── nav2_params.yaml          # AMCL, costmaps, MPPI, SmacPlanner2D, BT
│   └── collision_monitor.yaml    # CollisionMonitor + VelocitySmoother
├── bt_xml/
│   └── navigate_w_replanning_and_recovery.xml  # custom BT with progressive recovery
├── launch/
│   ├── bringup.launch.py         # full stack (nav2 + pc_to_laserscan + static TF)
│   └── mission.launch.py         # mission_runner + estop_node
├── nav2_tracer_agilex/
│   ├── mission_runner.py         # waypoint loop via nav2_simple_commander
│   └── estop_node.py             # GPIO E-stop → cancel nav + zero velocity
└── maps/                         # place GLIM-generated .pgm + .yaml here
```

---

## Dependencies

**ROS2 Humble packages** (install on Jetson):
```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-nav2-smac-planner \
  ros-humble-nav2-rotation-shim-controller \
  ros-humble-nav2-collision-monitor \
  ros-humble-nav2-velocity-smoother \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-spatio-temporal-voxel-layer
```

**FAST-LIO2** (odometry — manual install):
```bash
cd ~/ros2_ws/src
git clone https://github.com/hku-mars/FAST_LIO --recursive
cd .. && colcon build --packages-select fast_lio
```

**Jetson.GPIO** (for E-stop node):
```bash
sudo pip3 install Jetson.GPIO
sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER
```

---

## Build & Run

```bash
# Clone on Jetson
cd ~/ros2_ws/src
git clone https://github.com/Narcis-Abella/nav2-tracer-agilex.git nav2_tracer_agilex

# Build
cd ~/ros2_ws
colcon build --packages-select nav2_tracer_agilex
source install/setup.bash

# Copy your map files (generated with GLIM day before)
cp /path/to/map.pgm /path/to/map.yaml \
  ~/ros2_ws/src/nav2_tracer_agilex/maps/

# Launch full stack
ros2 launch nav2_tracer_agilex bringup.launch.py \
  map:=$HOME/ros2_ws/src/nav2_tracer_agilex/maps/map.yaml

# Launch mission (separate terminal)
ros2 launch nav2_tracer_agilex mission.launch.py
```

---

## Critical Parameters to Validate On-Site

| Parameter | Location | Notes |
|-----------|----------|-------|
| `footprint` | `nav2_params.yaml` | Measure Tracer+chair after real mounting |
| `inflation_radius` | `nav2_params.yaml` | Must keep 80cm doors passable — start at 0.07m |
| `lidar_z` (static TF) | `bringup.launch.py` | Measure actual sensor height |
| `lidar_x/y/yaw` | `bringup.launch.py` | Measure actual sensor offset |
| FAST-LIO2 stability | runtime | `htop` + `tegrastats` during turns — fallback: KISS-ICP |
| `decay_acceleration` | `nav2_params.yaml` | Verify dynamic robot clears fast when seen |
| Spin recovery | runtime | Verify 90° turn fits in 120cm corridor |
| `movement_time_allowance` | `nav2_params.yaml` | Tune to observed obstacle dwell time |
| `batch_size` MPPI | `nav2_params.yaml` | Monitor CPU: 2000 samples @30Hz on Jetson |

---

## Behavior Tree Recovery Escalation

1. **ConditionalReplanning** — 1Hz rate-limited replan + path validity check
2. **ClearGlobalCostmap** — on compute path failure
3. **ClearLocalCostmap** — on follow path failure
4. **SoftRecovery** — Wait(2s) + ClearLocal → dynamic obstacle passed
5. **ActiveRecovery** — BackUp(0.3m) + Spin(90°) + ClearBoth → physically unstuck
6. **LastResort** — Wait(5s) + ClearBoth → slow dynamic obstacle

---

## Waypoints

Edit `WAYPOINTS_XYZ` in `nav2_tracer_agilex/mission_runner.py` with coordinates
from the GLIM map (measured day before challenge):

```python
WAYPOINTS_XYZ = [
    (x, y, yaw_degrees),
    ...
    (0.0, 0.0, 180.0),  # home
]
```

---

## E-Stop

Physical button → GPIO pin 18 (Jetson BOARD numbering, configurable).
Pressed: cancels active navigation goal + publishes zero velocity.
Released: navigation ready for new goals (no auto-resume).
