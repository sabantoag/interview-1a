# Agri-Robotics Interview Exercises

Two take-home-in-an-interview ROS2 exercises built around an autonomous tractor
domain (fields, implements, telemetry). Each exercise ships equivalent C++ and
Python starter packages so you can run whichever language the candidate
prefers. Candidates clone this repo, build it locally, and edit a small number
of files marked with `TODO`.

- **Q1 -- Geometry Overlap** (`src/q1_geometry_overlap/`): compare/overlap
  geometries (field boundary, restricted zones, implement footprint) coming
  from an internal "geometry provider" node.
- **Q2 -- Safety Monitor** (`src/q2_safety_monitor/`): monitor streaming
  tractor telemetry, flag out-of-range values with debouncing, and check a
  geofence, publishing standard ROS2 diagnostics.

Each question is self-contained -- you can run either one independently.

## Target audience / timing

Mid-level engineers. Budget ~45 minutes per question, including a few minutes
up front to look at the provided (non-TODO) code and a few minutes at the end
for the performance/design discussion questions in each question's README.

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
interviewer_solutions/        <- Reference solutions + grading notes.
                                 Remove this folder (or keep it out of the
                                 fork/branch you hand to candidates!) before
                                 sending the repo out.
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

## Before you use this in a real interview

The ROS2 C++ packages in this repo were written carefully against standard
`rclcpp`/`ament_cmake` patterns but have **not** been `colcon build`-verified
in a real ROS2 environment (this authoring environment doesn't have ROS2
installed). Do a full `colcon build` + `ros2 launch` dry run yourself first --
budget 15-20 minutes to fix any small build issues before the first real
interview. Python files have been syntax-checked; XML manifests have been
validated.

## Handing this to a candidate

1. Delete or branch off `interviewer_solutions/` before sharing.
2. Point them at Option A or B above for setup instructions.
3. Point them at `src/q1_geometry_overlap/README.md` and/or
   `src/q2_safety_monitor/README.md` for the actual prompt.
