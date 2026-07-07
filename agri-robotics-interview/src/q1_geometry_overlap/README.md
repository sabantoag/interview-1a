# Q1 -- Geometry Overlap (field / restricted-zone / implement footprint)

**Target:** mid-level engineer, ~45 minutes.
**Languages:** C++ (`geometry_overlap_cpp/`, using **Boost.Geometry**) and
Python (`geometry_overlap_py/`, using **Shapely**), functionally identical.

This exercise is deliberately about using a real geospatial library
*correctly*, not about implementing point-in-polygon or polygon clipping by
hand. If you want an algorithmic (no-library) version of this exercise
instead, that's a different variant -- this one tests library fluency,
correct geometry construction, and defensive/performance thinking around
library calls.

## Scenario

An autonomous tractor is running an implement (e.g. a sprayer) through a
field. An internal ROS2 node (`geometry_provider_node`, already
implemented -- this is your "internal node that returns geometries")
publishes three pieces of geometry as the tractor drives:

- `/field/boundary` -- the field's outer boundary polygon (published once,
  latched).
- `/field/restricted_zone` -- a no-spray buffer zone inside the field, e.g.
  around a waterway (published once, latched).
- `/tractor/implement_footprint` -- the implement's current footprint on
  the ground, republished as the tractor moves (~2 Hz).

The candidate's job is to fill in `geometry_types.{hpp,cpp}` (C++) /
`geometry_types.py` (Python) -- used by a second node
(`overlap_checker_node`, wiring already provided) -- to answer two
questions on every update:

1. **Is the implement currently inside the restricted zone?** (boolean --
   core, required task)
2. **How much of the implement's footprint overlaps the field boundary,
   in square meters?** (also required -- this is a one-liner once you have
   the library polygons built, not a stretch goal like it was in the
   hand-rolled version of this exercise)

## What's given vs. what candidates write

| File | Status |
|---|---|
| `geometry_types.{hpp,cpp}` / `geometry_types.py` | **Candidate edits this.** `compute_bounding_box`/`bounding_boxes_overlap` are given as an optional prefilter. `to_boost_polygon`/`to_shapely_polygon`, `polygons_intersect`, and `polygon_intersection_area` are `TODO` (required). `find_intersecting_zones` is `TODO` (stretch -- spatial indexing with an R-tree/STRtree). |
| `geometry_provider_node.*` | Given. Publishes mock geometry on a deterministic path. |
| `overlap_checker_node.*` | Given. ROS wiring only -- subscribes, calls the candidate's functions, publishes results. Untouched by the library swap. |
| `test/test_geometry_types.*` | Given. Plain assert/pytest tests, runnable without ROS (see below) -- candidates can use these to self-check as they go. |

## Required reading before the interview

- C++: candidates need to know Boost.Geometry's polygon type expects a
  **closed** ring (first point repeated at the end) and a specific winding
  order (clockwise, by default) -- and that `boost::geometry::correct()`
  will fix the winding for you once the ring is loaded. This is really the
  crux of the required part of the C++ exercise; if a candidate is stuck,
  this is the hint to give.
- Python: Shapely is much more forgiving (no winding/closure requirements
  to think about), so the Python version is meaningfully faster to finish
  -- expect Python candidates to have more time left for the stretch goal
  than C++ candidates. Calibrate accordingly; this isn't a bug in the
  exercise, it's a real difference between the two ecosystems worth
  discussing.

## Running it

```bash
# One-time setup (see the top-level README for details):
#   sudo apt install libboost-dev
#   pip install --break-system-packages "shapely>=2.0"

colcon build --symlink-install --packages-select geometry_overlap_cpp geometry_overlap_py
source install/setup.bash

# C++:
ros2 launch geometry_overlap_cpp q1_demo.launch.py
# Python:
ros2 launch geometry_overlap_py q1_demo.launch.py
```

In another terminal:

```bash
ros2 topic echo /safety/restricted_zone_violation
ros2 topic echo /field/coverage_overlap_area_m2
```

The mock tractor drives a fixed loop (see `geometry_provider_node`): it
starts outside the field, drives through it, clips into the restricted
zone for a few ticks, then exits the field again. You should see
`restricted_zone_violation` flip `true` for the few ticks where it's in the
buffer zone, and the coverage area rise as it enters the field, peak while
fully inside, and drop back to 0 as it leaves.

You don't need the ROS graph running at all to iterate on the algorithms
themselves:

```bash
# C++ (no colcon needed for this part, just libboost-dev):
g++ -std=c++17 -Iinclude src/geometry_types.cpp test/test_geometry_types.cpp -o /tmp/test_geo && /tmp/test_geo
# Python:
python3 -m pytest test/test_geometry_types.py -v
```
