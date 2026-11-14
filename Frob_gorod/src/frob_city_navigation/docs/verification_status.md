# Frob City verification status — WP13

This file records verification level, not physical authorization.

| Gate | Scope | Status after package creation |
|---|---|---|
| G01–G12 | Prior frozen/core/component gates | Expected already green in target workspace |
| G13.1 | readiness + launch contracts | Run by `verify_wp13.zsh` |
| G13.2 | command-chain/replay isolation static audit | Run by `verify_wp13.zsh` |
| G13.3 | ROS build | Run on Raspberry Pi |
| G13.4 | live headless replay with fake neighbors | Run on Raspberry Pi; no motor topic |
| G13.5 | full source regression | Run on Raspberry Pi |
| G13.6 | isolated colcon regression | Run on Raspberry Pi |
| G14 | full mission integration/replay/sim | NOT YET CLAIMED |
| G15 | physical calibration/commissioning | NOT YET CLAIMED |

A G13 PASS means one headless launch, readiness, diagnostics, logging selection, and command-topic ownership are component-tested with hardware isolated. It does not validate real detector accuracy, real Nav2 tuning, surveyed field geometry, braking distance, or motor behavior.
