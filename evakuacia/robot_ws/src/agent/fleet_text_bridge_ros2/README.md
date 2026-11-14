# fleet_text_bridge_ros2

MQTT ↔ ROS 2 bridge for the SVERH fleet protocol. The package has no robot YAML configuration. It reads identity and server settings from exported environment variables.

```bash
export FLEET_ROBOT_ID='rover-01'
export FLEET_SERVER_IP='10.194.179.111'
export FLEET_MQTT_HOST="$FLEET_SERVER_IP"
export FLEET_MQTT_PORT='1883'
```

Build and launch:

```bash
colcon build --symlink-install --packages-up-to fleet_text_bridge_ros2
source install/setup.bash
ros2 launch fleet_text_bridge_ros2 bridge.launch.py
```

Full bridge + agent stack:

```bash
ros2 launch fleet_text_bridge_ros2 rover_agent_stack.launch.py
```
