# Rover maps

- `current/` is the map used by default by Nav2.
- `archive/` contains previous map versions.
- `map.yaml` and its image are used by Nav2 Map Server and AMCL.
- `map.posegraph` and `map.data` are used by SLAM Toolbox to continue mapping.

Save a map while SLAM is running:

```bash
ros2 run rover_navigation rover_map save room
```

The command updates the source project and synchronizes the installed current
map so navigation can be launched immediately without rebuilding.
