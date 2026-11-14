# Rover Bringup Configuration

This directory is the source of truth for the assembled rover. Package-level
configs are kept as `*.default.example.yaml` files only so each package can be
launched and understood in isolation.

## Layout

- `rover_v1.yaml`: physical robot identity, geometry, wheel layout and calibration.
- `profiles/`: which components are enabled for a launch mode.
- `components/`: runtime parameters for each hardware, discovery or UI component.
- `components/agent.yaml`: local MCP/LLM agent and MQTT fleet bridge settings.
- `topics.yaml`: shared topic names and TF frame names.
- `localization/`: EKF parameter files used by `rover_bringup`.
- `navigation/`: Nav2 and SLAM parameter files used by `rover_bringup`.

Component configs can use explicit references to already loaded bringup config
values. For example, `@robot.id` resolves from `rover_v1.yaml`, and
`@topics.cmd_vel_test` resolves from `topics.yaml`. `@env.NAME` resolves from an
environment variable and becomes an empty string when unset.

## Precedence

Normal launch configuration is resolved in this order:

1. Package node defaults.
2. Package `config/*.default.example.yaml` when a package is launched standalone.
3. `rover_bringup/config/components/*.yaml`.
4. `rover_bringup/config/rover_v1.yaml` for robot-specific geometry and
   calibration.
5. Explicit launch arguments, for example `use_lidar:=false`.

If the same setting appears in multiple places, prefer moving the real rover
value into this directory and leaving the package config as a generic example.

## Common Launches

```bash
ros2 launch rover_bringup robot.launch.py profile:=full
ros2 launch rover_bringup robot.launch.py profile:=minimal
ros2 launch rover_bringup robot.launch.py profile:=agent
ros2 launch rover_bringup robot.launch.py profile:=hardware
ros2 launch rover_bringup robot.launch.py profile:=navigation
ros2 launch rover_bringup robot.launch.py profile:=mapping
```

The `navigation` profile enables Nav2 through `components.nav2: true`; the
`mapping` profile enables SLAM Toolbox through `components.slam: true`.

Per-component overrides are still available from the command line:

```bash
ros2 launch rover_bringup robot.launch.py profile:=full use_camera:=false
ros2 launch rover_bringup robot.launch.py profile:=mapping use_slam:=false
```
