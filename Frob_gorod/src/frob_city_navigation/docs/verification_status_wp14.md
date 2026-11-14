# WP14 Verification Status

WP14 is the headless integration/replay/simulation gate. It does not commission physical geometry, braking, camera calibration, motor hardware, or competition-speed operation.

| Gate | Property | Hardware | Status before robot run |
|---|---|---:|---|
| G14.1 | Contracts + deterministic pure replay | No | LOCAL PASS |
| G14.3 | Full production replay stack starts held; no motor topic | No | RUN ON TARGET |
| G14.4 | Real map_planner unresolved gate -> qualified Observe -> legal replan -> idempotent report | No | RUN ON TARGET |
| G14.5 | Real navigator + real motion_guard: 2 unique bus stops -> parking -> DONE | No | RUN ON TARGET |
| G14.6 | Nav2 reject/abort, stale odom, map identity change, readiness loss all fail closed | No | RUN ON TARGET |
| G14.7 | 20 identical fixed mission replays produce one identical decision trace | No | LOCAL PASS |
| G14.8 | Real Nav2 controller follows a path against kinematic fake base | No | RUN ON TARGET |
| G14.9/10 | Full pytest + isolated colcon regression | No | RUN ON TARGET |

A G14 PASS authorizes only controlled WP15 physical commissioning. It is not evidence that real dimensions, detector range, braking distance, watchdog latency, or T_map_field are calibrated.
