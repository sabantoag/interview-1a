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

As with Q1, the core logic has no ROS dependency and can be iterated on
without the graph running:

```bash
# C++:
g++ -std=c++17 -Iinclude src/safety_types.cpp test/test_safety_types.cpp -o /tmp/test_safety && /tmp/test_safety
# Python:
python3 -m pytest test/test_safety_types.py -v
```

## What to look for

- **Class design for `ViolationTracker`.** Does the candidate land on
  something like a `map<string, int>` of consecutive-violation counts?
  Do they handle "never seen this name before" cleanly? Do they correctly
  reset the streak on a good reading, rather than just decrementing it?
- **Correct use of the debounce output in `evaluate()`.** A common bug:
  computing `in_range` correctly but comparing the *raw* value against
  thresholds directly instead of routing it through `ViolationTracker`,
  which defeats the whole point of debouncing. Watch for candidates who
  reason explicitly about *why* they need the debounce layer (noisy
  sensors, avoiding nuisance E-STOPs) rather than treating it as busywork.
- **Geofencing folded into the same mechanism.** Do they realize the
  geofence check can reuse the exact same `ViolationTracker.record()` /
  `Severity` machinery as the numeric checks, just keyed under a different
  name (e.g. `"geofence"`), rather than writing a separate one-off
  E-STOP path?
- **Aggregation logic.** Is "overall severity = worst of all individual
  severities" correct and clearly expressed, or an ad hoc pile of
  `if`/`elif`? Do they publish `/safety/estop` based on that aggregate,
  or accidentally base it on just the last parameter checked in a loop?
- **Cursory performance awareness** (see discussion questions below) --
  this doesn't need to be optimized code in 45 minutes, but they should be
  able to reason about it out loud.

## Follow-up / discussion questions (performance-focused)

- "This runs at 10 Hz with 5 numeric parameters plus the geofence. If we
  scaled to 50 sensors at 100 Hz, what in your `evaluate()` would you
  reconsider first -- and why?" (Look for: map lookups vs. array
  indexing, avoiding string formatting/allocation on the hot path,
  avoiding rebuilding the whole `DiagnosticArray` message from scratch
  if only one value changed, etc.)
- "In C++: are there any allocations happening inside `evaluate()` or the
  subscription callbacks that you'd want to eliminate for a hard
  real-time version of this node (e.g. `std::string` construction,
  `std::vector::push_back` reallocation, `unordered_map` lookups by
  string key)? How would you restructure to avoid them?"
- "In Python: `rclpy`'s default executor is single-threaded. What does
  that imply about a slow `evaluate()` call and telemetry-callback
  latency? When (if ever) would you reach for a multi-threaded executor
  here, and what would you need to make thread-safe if you did?"
- "Right now `ViolationTracker` stores a simple consecutive-violation
  counter per name. When would a counter be the *wrong* choice compared
  to, say, a sliding time window of the last N seconds of readings --
  especially if sensors don't publish at a perfectly regular rate?"
- "We hardcoded the safe ranges in the constructor. What's the tradeoff
  of moving these to ROS2 parameters (`declare_parameter`/
  `get_parameter`) instead -- what do you gain, and what new failure
  modes does it introduce (e.g. a bad parameter value at launch)?"
