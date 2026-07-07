# Q1 -- Geometry Overlap (field / restricted-zone / implement footprint)

**Target:** mid-level engineer, ~45 minutes.
**Languages:** C++ (`geometry_overlap_cpp/`) and Python (`geometry_overlap_py/`), functionally identical.

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

The candidate's job is to fill in the geometry algorithms
(`geometry_types.hpp`/`.cpp` in C++, `geometry_types.py` in Python) that a
second node (`overlap_checker_node`, wiring already provided) uses to
answer two questions on every update:

1. **Is the implement currently inside the restricted zone?** (boolean --
   this is the core, required task)
2. **Stretch goal:** how much of the implement's footprint currently
   overlaps the field boundary, in square meters?

## What's given vs. what candidates write

| File | Status |
|---|---|
| `geometry_types.{hpp,cpp}` / `geometry_types.py` | **Candidate edits this.** `compute_bounding_box`, `bounding_boxes_overlap`, `segments_intersect`, `polygon_area` are implemented as worked examples. `point_in_polygon` and `polygons_intersect` are `TODO` (required). `clip_convex_polygon` is `TODO` (stretch). |
| `geometry_provider_node.*` | Given. Publishes mock geometry on a deterministic path. Do not need to read closely, but useful context. |
| `overlap_checker_node.*` | Given. ROS wiring only -- subscribes, calls the candidate's functions, publishes results. |
| `test/test_geometry_types.*` | Given. Plain assert/pytest tests, runnable without ROS (see below) -- candidates can use these to self-check as they go. |

## Running it

```bash
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
ros2 topic echo /field/coverage_overlap_area_m2   # stretch goal
```

The mock tractor drives a fixed loop (see `geometry_provider_node`): it
starts outside the field, drives through it, clips into the restricted
zone for a few ticks, then exits the field again. You should see
`restricted_zone_violation` flip `true` for the few ticks where it's in the
buffer zone, and (if the stretch goal is implemented) the coverage area
rise as it enters the field, peak while fully inside, and drop back to 0 as
it leaves.

You don't need the ROS graph running at all to iterate on the algorithms
themselves -- the geometry code has no ROS dependency:

```bash
# C++ (no colcon needed for this part):
g++ -std=c++17 -Iinclude src/geometry_types.cpp test/test_geometry_types.cpp -o /tmp/test_geo && /tmp/test_geo
# Python:
python3 -m pytest test/test_geometry_types.py -v
```

## What to look for

- **Correctness of the point-in-polygon test.** Ray casting (even-odd
  rule) is the expected approach; winding number is also acceptable. Watch
  for off-by-one errors in the edge-wrap loop and for not handling the
  "ray is horizontal to an edge" degenerate case (not tested here, but
  worth asking about).
- **How they compose `polygons_intersect` from primitives.** The strong
  signal here is recognizing that vertex-in-polygon alone isn't sufficient
  (a thin bar can cross a square with no vertex of either inside the
  other) and that they need the edge-intersection check too. A candidate
  who jumps straight to "check every edge against every edge" without the
  bounding-box short-circuit or vertex check isn't necessarily wrong, but
  it's a good moment to ask about complexity (see below).
- **Use of the given primitives rather than reinventing them.** Do they
  reach for `bounding_boxes_overlap`/`segments_intersect` or rewrite
  similar logic inline? Reuse is a good sign of reading the codebase they
  were handed.
- **Whether they test as they go.** The unit tests are there to be run
  early and often -- candidates who write `point_in_polygon`, run the
  tests, then move on to `polygons_intersect` are working the way you'd
  want them to on your team.
- **Stretch: Sutherland-Hodgman clipping.** Don't expect most mid-level
  candidates to finish this cold in 45 minutes; partial credit for a
  correct plan (clip subject polygon against each edge of the convex
  clip polygon, keep the "inside" side) even if the implementation isn't
  finished.

## Follow-up / discussion questions

- "`polygons_intersect` assumes simple, non-self-intersecting polygons.
  What could go wrong if a caller passed in a self-intersecting polygon,
  and how would you defend against it?"
- "Right now `clip_convex_polygon` assumes `clip_convex` is convex and
  CCW. What would break if the restricted zone were a concave polygon
  (e.g. an L-shaped exclusion area), and how would you generalize this?"
- "The provider node re-publishes the whole footprint polygon every tick
  even though it's always the same 4-point square, just translated. In a
  real system with many implements, what would you change about this
  interface?"
- "If field boundaries had thousands of vertices (a detailed, surveyed
  boundary rather than a rectangle), which part of your solution would
  become the bottleneck, and how would you address it?"
