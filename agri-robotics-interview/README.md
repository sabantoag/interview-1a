# Agri-Robotics Interview Exercises

Two exercises built around an autonomous tractor domain (fields, implements,
telemetry). Each exercise ships equivalent C++ and Python starter packages so
you can run whichever language the candidate prefers. Candidates clone this
repo, build it locally, and edit a small number of files marked with `TODO`.

- **Q1 -- Geometry Overlap** (`src/q1_geometry_overlap/`): compare/overlap
  geometries (field boundary, restricted zones, implement footprint) coming
  from an internal "geometry provider" node.
- **Q2 -- Safety Monitor** (`src/q2_safety_monitor/`): monitor streaming
  tractor telemetry, flag out-of-range values with debouncing, and check a
  geofence, publishing standard ROS2 diagnostics.

## Prerequisites

Pick one:

### Option A -- Docker (recommended, fastest to get a candidate running)

```bash
docker build -t agri-robotics-interview .
docker run -it --rm -v $(pwd):/workspace agri-robotics-interview
# inside the container:
cd /workspace
colcon build --symlink-install
source install/setup.bash
```

A VS Code `.devcontainer` is also included if you prefer "Reopen in
Container."

### Option B -- Native install

- Ubuntu 22.04 + [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)
  (desktop install recommended). ROS2 Jazzy on Ubuntu 24.04 also works -- the
  code only uses long-stable APIs.
- `sudo apt install python3-colcon-common-extensions`

Then:

```bash
source /opt/ros/humble/setup.bash
cd agri-robotics-interview
colcon build --symlink-install
source install/setup.bash
```

## Repo layout

```
src/
  q1_geometry_overlap/        <- Question 1 (see its README.md for the prompt)
    geometry_overlap_cpp/     <- C++ ROS2 package, candidate edits TODOs here
    geometry_overlap_py/      <- Python ROS2 package, same TODOs
  q2_safety_monitor/          <- Question 2 (see its README.md for the prompt)
    safety_monitor_cpp/
    safety_monitor_py/
```

## Running a question (example: Q1, Python)

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch geometry_overlap_py q1_demo.launch.py
```

This brings up the given "provider" node (publishing mock field/tractor
geometry) alongside the candidate's node. Use `ros2 topic echo <topic>` in a
second terminal to watch results. Each question's README lists the exact
topics to watch.
