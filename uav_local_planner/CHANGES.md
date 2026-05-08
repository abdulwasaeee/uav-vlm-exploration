# Changes — Replan Disable + Bag Directory + Diag Fix (2026-05-08)

## Root Cause (from bag analysis: pole_bypass_20260508_180254 / 180341)

Two `pole_bypass` bags revealed that a mystery 4th `global_path` message was
arriving at t≈7-8 s (after the 3 from `send_and_record.py`), causing premature
`mission_complete` and collisions.

**Source:** `planner_server_node` (`uav_planner_interface/src/planner_server_node.cpp:203`),
left running from a previous launch or launched despite `with_global_planner:=false`.

**Chain of events:**
1. Drone moves toward (9,0,1.5), deviates >2 m from the straight-line path
2. Waypoint manager's replan check triggers `requestReplan()` → sends
   `NavigateToGoal` action to planner_server
3. With no OctoMap running, the planner returns a garbage short path (2.5 m /
   1.7 m) ending nowhere near the original goal
4. Waypoint manager processes the short path, reaches its end, declares
   `mission_complete=True` while drone is still 6 m / 2.6 m from the real target

**Secondary finding:** Diag messages for STALLED/ORBITING states published only
10 fields vs. 12 in normal cycles, causing misaligned columns in PlotJuggler.

## Fixes

### Fix A — Disable replan by default (`waypoint_manager_node.cpp`)

New `enable_replan` parameter (default `false`) gates the deviation→replan
check.  When the MP local planner is used standalone (no global planner), the
straight-line path from `send_and_record.py` is just a 2-pose reference —
obstacle avoidance necessarily deviates from it.  Replanning in that mode
produces garbage paths and sabotages the run.

### Fix B — Diag layout fix (`mp_node.cpp`)

STALLED and ORBITING hold states now publish the full 12-field diag layout
instead of 10 fields.  Added the missing `estop` (index 10) and
`obstacle_detected` (index 11) fields.

### Fix C — Dedicated bags directory

- Created `bags/` at workspace root for all ROS 2 bag recordings
- `send_and_record.py` now saves bags to `<ws>/bags/` automatically
- Added `bags/` to `.gitignore`

## Parameters Added

| Parameter | Default | Description |
|---|---|---|
| `enable_replan` | `false` | Allow waypoint manager to trigger replan on deviation |

## Files Modified

| File | Change |
|---|---|
| `src/waypoint_manager_node.cpp` | Added `enable_replan` param; gated replan check |
| `src/mp_node.cpp` | Fixed STALLED/ORBITING diag from 10 → 12 fields |
| `scripts/send_and_record.py` | Bag output now goes to `<ws>/bags/` |
| `.gitignore` | Added `bags/` |

---

# Changes — Avoidance Persistence (2026-05-08)

## Root Cause (from bag analysis: pole_bypass_20260508_171254)

Bag analysis of a pole-avoidance run revealed two compounding failures that let
the drone fly straight into an obstacle even when the sensor had detected it:

1. **No persistence on obstacle detection** — `obstacle_detected` is a raw
   threshold (`obs_dist < arc_length`).  In the bag, `obs_dist` dropped to
   1.991 m (below 2.0 m) for 8 planner cycles (0.4 s), putting the planner in
   AVOIDING state.  On the 9th cycle it ticked to 2.015 m and `obstacle_detected`
   flipped false immediately, returning the drone to full-speed NOMINAL.  With
   no hysteresis the planner oscillates between AVOIDING and NOMINAL whenever
   the obstacle sits near the 2.0 m boundary.

2. **No commitment to bypass** — even when bypass *would* engage (direct path
   blocked), the exit condition fires the moment any primitive near the goal
   direction becomes valid.  A thin obstacle that intermittently blocks/clears
   the forward cone causes bypass to exit in a single cycle, preventing the
   drone from committing to a clear side.

3. **Speed ramp too late** — the adaptive-speed ramp started at `arc_length / 2`
   (1.0 m), so the drone was still at full 2.5 m/s when it first detected the
   obstacle at 2.0 m.  The ramp now starts at the full detection range.

## Fixes

### Fix A — Obstacle detection hysteresis (`motion_primitives.cpp/.hpp`)

`obstacle_detected` now uses a two-level threshold:
- **Engage**: `obs_dist < arc_length` (2.0 m) — same as before
- **Disengage**: only when `obs_dist > arc_length × obstacle_hysteresis_factor`
  (default 2.6 m)

A new `prev_obstacle_detected_` member carries state between cycles.  The flag
is cleared in `reset()` on every new waypoint.

### Fix B — Bypass minimum hold (`motion_primitives.cpp/.hpp`)

A `bypass_hold_cycles_` counter increments each planner cycle that bypass is
active.  The exit condition (near-straight primitive toward goal becomes valid)
is only checked after `bypass_hold_cycles_ >= bypass_min_hold_cycles` (default
60 cycles = 3 s).

The **safety fallback** (bypass side fully blocked → revert to unrestricted) is
always active regardless of the timer — the drone can never get stuck.

### Fix C — Speed ramp starts at detection range (`motion_primitives.cpp`)

`d_high` in `adaptiveSpeed()` changed from `arc_length / 2` (1.0 m) to
`arc_length` (2.0 m).  Speed now ramps from max at 2.0 m down to min at
0.6 m (`min_clearance`), giving the full detection window to decelerate.

## Parameters Added

| Parameter | Default | Description |
|---|---|---|
| `obstacle_hysteresis_factor` | 1.3 | Disengage obstacle_detected at arc_length × factor (2.6 m) |
| `bypass_min_hold_cycles` | 60 | Minimum cycles (3 s) before bypass exit is checked |

## Files Modified

| File | Change |
|---|---|
| `include/uav_local_planner/motion_primitives.hpp` | Added `obstacle_hysteresis_factor`, `bypass_min_hold_cycles` to `MPConfig`; added `bypass_hold_cycles_`, `prev_obstacle_detected_` members |
| `src/motion_primitives.cpp` | Fix A hysteresis, Fix B hold counter, Fix C speed ramp; updated `reset()` |
| `src/mp_node.cpp` | Declared two new ROS2 parameters |
| `config/mp_params.yaml` | Added `obstacle_hysteresis_factor: 1.3`, `bypass_min_hold_cycles: 60` |

---

# Changes — Stall Detector Redesign (2026-05-08)

## Root Cause (from bag analysis)

The V3 fixes did not resolve the issue because the real failure mode was
**oscillation near the goal**, not a large-radius orbit:

- Drone reached 0.12 m from goal at t = 5.5 s, then bounced between 0.12 m
  and 1.75 m for 55 seconds without triggering any detector.
- Drone yaw change: max 1.6° total — no orbit at all.
- `stall_ignore_radius = 1.0 m` caused the stall detector to skip 92% of all
  samples (dist was below 1.0 m almost the entire run).
- Even with a smaller ignore radius, the old sliding-window stall logic still
  fails for oscillation: each new approach from 1.75 m → 0.3 m shows 1.45 m of
  "progress" in a 3-second window, so `initial - min < 0.3 m` is never true.

## Fix — StallDetector redesign (`mp_node.cpp`)

Replaced the sliding-window design with a **best-ever distance + improvement
timer**:

- `best_dist` = smallest dist-to-goal seen since last waypoint/reset
- Clock resets only when `dist < best_dist - progress_min` (0.15 m)
- `stalled` fires when clock exceeds `no_improve_window` (20 s)

With the logged data, this design fires stall at t = 25.7 s
(`best_dist = 0.27 m`, no improvement for 20 s) vs. never with the old design.

Correctly handles all cases:
- **Complete stop**: dist never improves → clock runs to 20 s → STALLED
- **Oscillation near obstacle**: best_dist = 0.12 m, bouncing never beats
  0.12 − 0.15 = −0.03 m → clock runs to 20 s → STALLED
- **Normal navigation**: dist continuously improves → clock keeps resetting
- **Slow detour around obstacle**: dist may increase temporarily, but the
  20 s window accommodates typical indoor detours before firing

## Parameters Changed

| Parameter | Old | New | Notes |
|---|---|---|---|
| `stall_window_sec` | 3.0 s | — | **removed** (sliding-window approach) |
| `stall_min_samples` | 45 | — | **removed** |
| `stall_no_improve_window` | — | 20.0 s | **new** — timeout since last improvement |
| `stall_progress_min` | 0.3 m | 0.15 m | smaller threshold catches gradual improvement |
| `stall_ignore_radius` | 1.0 m | 0.25 m | now matches waypoint acceptance radius |
| `orbit_ignore_radius` | 1.2 m | 0.3 m | was disabling orbit check near goal |

## Diagnostic update

`/uav/mp_diag` extended to 12 fields; indices 6–7 now carry stall internals:

| Index | Field |
|---|---|
| 6 | `stall_best_dist` (m) — closest approach since reset |
| 7 | `stall_since_improve` (s) — time since last best_dist improvement |
| 8 | `closest_obstacle_dist` (m) |
| 9 | `best_primitive_idx` |
| 10 | `estop` (0/1) |
| 11 | `obstacle_detected` (0/1) |

## Files Modified

| File | Change |
|---|---|
| `src/mp_node.cpp` | StallDetector complete redesign; updated params and diag layout |
| `config/mp_params.yaml` | Replaced stall params with new design |

---

# Changes — Orbit Mitigation V3 / Diagnostics (2026-05-08)

## Bugs Fixed

### Bug 1 (Critical) — OrbitDetector accumulator never reset (`mp_node.cpp`)

`min_dist` was updated to `dist_to_goal` **before** computing `progress`, so
`progress = min_dist - dist_to_goal` was always 0 and `yaw_accumulator` never
reset. The orbit detector therefore fired after any 270° of turning, including
legitimate goal-seeking. Fix: compute `progress` first, then update `min_dist`.

### Bug 2 (Major) — Bypass side committed in body frame, rotated with drone (`motion_primitives.cpp`)

The bypass filter used `end_point.y() > 0.05` (body-frame Y) to classify
"left" vs "right". As the drone yaws, body-frame left rotates in world-frame,
so the committed bypass direction changed continuously — exactly the behaviour
that drives an orbit. Fix: classify by
`normalizeAngle(terminal_az - goal_az_base)`, which is the signed angle of the
primitive relative to the goal direction and is invariant to drone yaw.

### Bug 3 (Minor) — Second bypass trigger was dead code (`motion_primitives.cpp`)

`bool goal_blocked = (near_goal_valid < 2) || (best_d_goal > marginal_angle)`:
`best_d_goal` tracked the minimum `daz` across all valid primitives. If any
primitive is within 25° of goal, `best_d_goal <= 25° < 30°`, making the second
condition always false when the first is false. Removed the dead condition.

### Bug 4 (Minor) — Bypass exit threshold tighter than entry (`motion_primitives.cpp`)

Entry threshold: `block_angle_rad = 25°`. Exit threshold was hardcoded to
`0.35 rad ≈ 20°`. A primitive reopening at 22° would fail to exit bypass and
immediately re-trigger entry on the next cycle. Fixed: exit now uses
`block_angle_rad` (25°) for a symmetric hysteresis boundary.

## Diagnostics Added (`mp_node.cpp`)

New publisher `/uav/mp_diag` (`std_msgs/Float64MultiArray`, 20 Hz).
Layout (by index):

| Index | Field | Units |
|---|---|---|
| 0 | dist_to_goal | m |
| 1 | yaw | rad |
| 2 | yaw_accumulator | rad |
| 3 | orbiting | 0/1 |
| 4 | stalled | 0/1 |
| 5 | bypass_state | 0=NONE 1=LEFT 2=RIGHT |
| 6 | closest_obstacle_dist | m (−1 if unknown) |
| 7 | best_primitive_idx | int (−1 if none) |
| 8 | estop | 0/1 |
| 9 | obstacle_detected | 0/1 |

Published at every non-trivial cycle, including STALLED and ORBITING holds.

## Script Added (`scripts/send_and_record.py`)

Python helper: sends a waypoint and records a ROS 2 bag until the drone stops.

```
python3 scripts/send_and_record.py --x 5.0 --y 0.0 --z 1.5
python3 scripts/send_and_record.py --x 5.0 --y 0.0 --z 1.5 --bag-prefix obs_run --settle-sec 5
```

Recording stops when `/uav/vfh_status` holds IDLE/STALLED/ORBITING for
`--settle-sec`, or when `cmd_vel ≈ 0` for that long after the drone moved.

## Files Modified (V3)

| File | Change |
|---|---|
| `src/mp_node.cpp` | Bug 1 fix; add `/uav/mp_diag` diagnostic publisher |
| `src/motion_primitives.cpp` | Bug 2, 3, 4 fixes in bypass state machine |
| `scripts/send_and_record.py` | New — waypoint send + bag recording helper |

---

# Changes — Orbit Mitigation (2026-05-08)

## Problem

When a waypoint was placed at or behind an obstacle, the drone entered a
persistent circular/orbital motion instead of stopping or navigating around.
Root cause: the cost function perpetually chased the "least blocked" horizontal
direction toward the goal, which rotated as the drone yawed, creating a feedback
loop.

## Solution: Two-Layer Mitigation

### Layer 1 — Orbit Detector (`mp_node.cpp`)

New `OrbitDetector` struct that accumulates yaw change over time. If the drone
rotates past 270° (configurable) without making 0.3 m of goal progress, an
orbit is declared and the drone hovers in place.

- **File:** `src/mp_node.cpp` (struct + integration in `MPNode::update()`)
- **Config:** `config/mp_params.yaml` (new `orbit_*` parameters)
- **Status topic:** publishes `"ORBITING"` on `/uav/vfh_status`

### Layer 2 — Bypass State Machine (`motion_primitives.cpp`)

When the direct path toward the goal is blocked (no valid primitive within ±25°
of the goal direction), the planner commits to bypassing consistently either
left or right. During bypass, only primitives on the committed side are
eligible for scoring. Bypass exits when a near-straight primitive toward the
goal becomes clear.

- **File:** `include/uav_local_planner/motion_primitives.hpp` (new `BypassState` enum, member, getter)
- **File:** `src/motion_primitives.cpp` (bypass logic in `update()`, reset in `reset()`)
- **Status topic:** publishes `"AVOIDING_L"` / `"AVOIDING_R"` during active bypass

## Parameters Added

| Parameter | Default | Description |
|---|---|---|
| `orbit_yaw_threshold` | 270.0° | Cumulative yaw change to trigger orbit detection |
| `orbit_progress_min` | 0.3 m | Goal progress that resets the yaw accumulator |
| `orbit_ignore_radius` | 1.2 m | Skip orbit check within this distance of goal |

## Files Modified

| File | Change |
|---|---|
| `include/uav_local_planner/motion_primitives.hpp` | Added `BypassState` enum, member, and `bypassState()` getter |
| `src/motion_primitives.cpp` | Bypass state machine in `update()`, clear in `reset()` |
| `src/mp_node.cpp` | Added `OrbitDetector` struct, integration in `MPNode` |
| `config/mp_params.yaml` | Added `orbit_*` parameters |

---

# Changes — Orbit Mitigation V2 (2026-05-08)

## Problems Found During Testing

The V1 orbit detector and bypass state machine didn't fully prevent circular
motion. Three root causes were identified:

1. **Orbit detector self-reset**: The yaw accumulator reset whenever the drone
   swung 0.3 m closer to the goal during its orbit. Since an orbit naturally
   oscillates distance-to-goal, the accumulator repeatedly reset before
   reaching the 270° trigger.

2. **Bypass triggered too late**: A single marginally-valid primitive within
   ±25° of the goal heading prevented bypass engagement. That primitive's arc
   would curve away from the goal, and by the next planner iteration the
   heading had changed enough to pick a different primitive — driving the orbit.

3. **No positional cost**: The scoring function only considered heading
   alignment (`d_goal`, `d_prev`). A primitive pointing toward the goal but
   curving 90° sideways scored better than one 30° off-goal that ended 2 m
   closer to the target.

## Solution: Three Additional Fixes

### Fix 1 — Orbit Detector Uses Best-Ever Distance (`mp_node.cpp`)

Replaced `start_dist` with `min_dist` (tracked since last reset). The yaw
accumulator now only resets when the drone gets `progress_min` closer than
the **best-ever** distance seen, not just closer than the start of the
current accumulation window. Distance oscillations during an orbit can no
longer reset the accumulator.

- **File:** `src/mp_node.cpp` (`OrbitDetector` struct)

### Fix 2 — Dual Bypass Triggers (`motion_primitives.cpp`)

Instead of requiring zero valid primitives within ±25° of the goal, bypass
now engages when **either**:
- Fewer than 2 valid primitives are within ±25° of the goal heading, **or**
- The single best-aligned valid primitive is more than 30° off the goal

- **File:** `src/motion_primitives.cpp` (bypass state machine, §7)

### Fix 3 — Positional Scoring Term (`motion_primitives.cpp`)

Added a `w_pos` weight to the cost function. For each valid primitive, the
endpoint is transformed to map frame and its XY distance to the goal is
computed. These distances are normalised against the best (closest) endpoint
distance across all valid primitives:

```
pos_norm = end_dist[i] / best_end_dist
cost = w_goal * d_goal + w_prev * d_prev + w_pos * pos_norm
```

This rewards primitives that actually move the drone closer to the goal, not
just those whose terminal heading aligns with the goal direction.

- **File:** `include/uav_local_planner/motion_primitives.hpp` (new `w_pos` member, default 2.0)
- **File:** `src/motion_primitives.cpp` (positional pre-pass + normalised cost, §8)
- **File:** `src/mp_node.cpp` (new `w_pos` parameter declaration)

## Parameters Added (V2)

| Parameter | Default | Description |
|---|---|---|
| `w_pos` | 2.0 | Weight for positional progress toward goal in cost function |

## Files Modified (V2)

| File | Change |
|---|---|
| `src/mp_node.cpp` | OrbitDetector: `start_dist` → `min_dist` (best-ever tracking) |
| `src/motion_primitives.cpp` | Dual bypass triggers, positional scoring pre-pass |
| `include/uav_local_planner/motion_primitives.hpp` | Added `w_pos` default (2.0) |
| `config/mp_params.yaml` | Added `w_pos` parameter |
