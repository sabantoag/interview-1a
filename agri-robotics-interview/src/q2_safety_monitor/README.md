# Q2 -- Robot Safety Monitor (out-of-range telemetry + geofence)

**Target:** mid-level engineer, ~45 minutes.
**Languages:** C++ (`safety_monitor_cpp/`) and Python (`safety_monitor_py/`), functionally identical.

## Scenario

A tractor streams telemetry over ROS2 topics: hydraulic pressure, engine
temperature, ground speed, PTO RPM, roll angle, and position. A given node
(`telemetry_publisher_node`) simulates this stream, including a
deterministic, repeating 30-second fault schedule (documented in that
file) so behavior is easy to reason about in a live demo:

- a **transient** ~0.5s spike in engine temperature (should surface as a
  WARNING at most -- not a full E-STOP),
- a **sustained** ~1.5s ground-speed excursion above the safe max (should
  escalate to CRITICAL and trigger E-STOP),
- a **sustained** ~1.5s excursion of the tractor's position outside the
  field boundary (a geofence violation, also CRITICAL/E-STOP).

The candidate builds a `safety_monitor_node` that:

1. Checks each telemetry value against a configured safe range.
2. **Debounces:** requires several consecutive out-of-range readings
   before escalating severity, so sensor noise / a single bad reading
   doesn't trip an E-STOP (see the transient-fault case above).
3. Checks a **geofence:** is the tractor's position inside the field
   boundary polygon (reusing the same point-in-polygon technique as Q1)?
4. Publishes results as a standard `diagnostic_msgs/DiagnosticArray` on
   `/diagnostics`, and a `std_msgs/Bool` E-STOP signal on `/safety/estop`.

## What's given vs. what candidates write

| File | Status |
|---|---|
| `safety_types.{hpp,cpp}` / `safety_types.py` | `Range`, `Severity`, and `point_in_polygon` are given. **`ViolationTracker` is the core `TODO`** -- candidates design and implement the debounce/streak-tracking class. Its public interface (constructor + `record()` + `consecutive_violations()`) is fixed since other files depend on it; internals are entirely theirs. |
| `telemetry_publisher_node.*` | Given. Simulates telemetry + the fault schedule described above. |
| `safety_monitor_node.*` | Mostly given (subscriptions, publishers, timer wiring). **The `evaluate()` method is `TODO`** -- this is where candidates actually *use* their `ViolationTracker`, check the geofence, build the diagnostics message, and decide on E-STOP. |
| `test/test_safety_types.*` | Given. Plain assert/pytest tests for `ViolationTracker`'s debounce behavior, runnable without ROS. |

## Running it

```bash
colcon build --symlink-install --packages-select safety_monitor_cpp safety_monitor_py
source install/setup.bash

# C++:
ros2 launch safety_monitor_cpp q2_demo.launch.py
# Python:
ros2 launch safety_monitor_py q2_demo.launch.py
```

In another terminal:

```bash
ros2 topic echo /diagnostics
ros2 topic echo /safety/estop
```

Over a 30-second window you should see: a WARNING-level `engine_temp_c`
entry around t=7.5s that does *not* trigger `/safety/estop` (if debounce is
implemented correctly); an ERROR-level `ground_speed_mps` entry and
`/safety/estop` going `true` around t=20s; and an ERROR-level `geofence`
entry with another `/safety/estop` around t=25s.

The core logic has no ROS dependency and can be iterated on by simply running:

```bash
# C++:
g++ -std=c++17 -Iinclude src/safety_types.cpp test/test_safety_types.cpp -o /tmp/test_safety && /tmp/test_safety
# Python:
python3 -m pytest test/test_safety_types.py -v
```
