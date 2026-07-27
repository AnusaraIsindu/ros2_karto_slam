# ros2_karto_slam

A ROS 2 port of `slam_karto` (Open Karto SLAM) with a companion `tb4_cpp_prac3` package implementing a person/wall-following behavior for TurtleBot 4.

## Packages

- **slam_karto** — ROS 2 SLAM node based on Open Karto, with scan matching and loop closure.
- **tb4_cpp_prac3** — Person and wall follower node for TurtleBot 4.
- **sparse_bundle_adjustment_ros2** — Dependency used by `slam_karto` for pose graph optimization (submodule).

## Prerequisites

- ROS 2 (Humble or newer)
- `colcon`

## Setup

```bash
git clone --recurse-submodules https://github.com/AnusaraIsindu/ros2_karto_slam.git
cd ros2_karto_slam/karto_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

If you already cloned without `--recurse-submodules`, run:

```bash
git submodule update --init --recursive
```

## Usage

Launch SLAM:

```bash
ros2 launch slam_karto slam_karto_sim_launch.py
```

Key parameters live in `karto_ws/src/slam_karto/slam_karto/config/karto_config.yaml` (frames, resolution, loop closure thresholds, etc.).
