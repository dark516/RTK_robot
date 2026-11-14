from __future__ import annotations

from collections import deque
import json
import os
import queue
import time
from typing import Any

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .duplicate_cache import DuplicateCache
from .message_codec import (
    decode_json_object,
    normalize_agent_payload,
    validate_command,
    validate_outgoing,
)


class FleetTextBridge(Node):
    """MQTT ↔ ROS 2 bridge for the rover text agent protocol.

    Commands are serialized before publication to `/agent/text_command`. This
    preserves unambiguous request/response correlation and also allows the
    bridge to normalize legacy plain-text agent answers.
    """

    def __init__(self) -> None:
        super().__init__("fleet_text_bridge")

        self.declare_parameter("robot_id", os.getenv("FLEET_ROBOT_ID", "rover-01"))
        self.declare_parameter("mqtt_host", os.getenv("FLEET_MQTT_HOST", os.getenv("FLEET_SERVER_IP", "127.0.0.1")))
        self.declare_parameter("mqtt_port", int(os.getenv("FLEET_MQTT_PORT", "1883")))
        self.declare_parameter("mqtt_topic_prefix", os.getenv("FLEET_MQTT_TOPIC_PREFIX", "fleet/v1/robots"))
        self.declare_parameter("mqtt_username", os.getenv("FLEET_MQTT_USERNAME", ""))
        self.declare_parameter("mqtt_password_env", os.getenv("FLEET_MQTT_PASSWORD_ENV", "FLEET_MQTT_PASSWORD"))
        self.declare_parameter("command_topic", os.getenv("AGENT_TEXT_COMMAND_TOPIC", "/agent/text_command"))
        self.declare_parameter("answer_topic", os.getenv("AGENT_ANSWER_TOPIC", "/agent/answer"))
        self.declare_parameter("status_topic", os.getenv("AGENT_STATUS_TOPIC", "/agent/status"))
        self.declare_parameter("duplicate_cache_size", int(os.getenv("FLEET_DUPLICATE_CACHE_SIZE", "100")))
        self.declare_parameter("agent_command_timeout_sec", float(os.getenv("FLEET_AGENT_COMMAND_TIMEOUT_SEC", "300")))

        self.robot_id = str(self.get_parameter("robot_id").value).strip()
        self.prefix = str(self.get_parameter("mqtt_topic_prefix").value).rstrip("/")
        self.command_mqtt_topic = f"{self.prefix}/{self.robot_id}/command"
        self.answer_mqtt_topic = f"{self.prefix}/{self.robot_id}/answer"
        self.status_mqtt_topic = f"{self.prefix}/{self.robot_id}/status"
        self.availability_mqtt_topic = f"{self.prefix}/{self.robot_id}/availability"
        self.agent_command_timeout_sec = float(
            self.get_parameter("agent_command_timeout_sec").value
        )

        self._incoming: queue.Queue[dict[str, Any]] = queue.Queue()
        self._pending: deque[dict[str, Any]] = deque()
        self._active_command: dict[str, Any] | None = None
        self._active_since: float | None = None
        self._duplicates = DuplicateCache(
            int(self.get_parameter("duplicate_cache_size").value)
        )

        self._command_pub = self.create_publisher(
            String,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("answer_topic").value),
            self._on_ros_answer,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self._on_ros_status,
            10,
        )
        self.create_timer(0.05, self._process_commands)
        self.create_timer(1.0, self._check_active_timeout)

        self._mqtt = self._make_mqtt_client()
        username = str(self.get_parameter("mqtt_username").value).strip()
        password_env = str(self.get_parameter("mqtt_password_env").value)
        if username:
            self._mqtt.username_pw_set(username, os.getenv(password_env, os.getenv("FLEET_MQTT_PASSWORD", "")) or None)
        self._mqtt.will_set(
            self.availability_mqtt_topic,
            json.dumps({"robot_id": self.robot_id, "online": False}),
            qos=1,
            retain=True,
        )
        self._mqtt.on_connect = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect
        self._mqtt.on_message = self._on_mqtt_message
        self._mqtt.reconnect_delay_set(min_delay=1, max_delay=15)

        host = str(self.get_parameter("mqtt_host").value)
        port = int(self.get_parameter("mqtt_port").value)
        self._mqtt.connect_async(host, port, 30)
        self._mqtt.loop_start()
        self.get_logger().info(
            f"Bridge {self.robot_id} connecting to MQTT {host}:{port}"
        )
        self.get_logger().info(
            "Agent protocol: JSON commands, correlated JSON status/answers; "
            "legacy plain agent output is normalized automatically."
        )

    def _make_mqtt_client(self):  # noqa: ANN202
        kwargs = {
            "client_id": f"bridge-{self.robot_id}",
            "clean_session": True,
        }
        try:
            return mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                **kwargs,
            )
        except (AttributeError, TypeError):
            return mqtt.Client(**kwargs)

    def _on_mqtt_connect(
        self,
        client,
        userdata,
        flags,
        rc,
        properties=None,
    ) -> None:  # noqa: ANN001
        if rc != 0:
            self.get_logger().error(f"MQTT connect failed rc={rc}")
            return
        client.subscribe(self.command_mqtt_topic, qos=1)
        client.publish(
            self.availability_mqtt_topic,
            json.dumps({"robot_id": self.robot_id, "online": True}),
            qos=1,
            retain=True,
        )
        self.get_logger().info(f"MQTT connected; subscribed to {self.command_mqtt_topic}")

    def _on_mqtt_disconnect(
        self,
        client,
        userdata,
        rc,
        properties=None,
    ) -> None:  # noqa: ANN001
        if rc != 0:
            self.get_logger().warning(f"Unexpected MQTT disconnect rc={rc}; reconnecting")

    def _on_mqtt_message(self, client, userdata, message) -> None:  # noqa: ANN001
        try:
            data = decode_json_object(message.payload)
            validate_command(data, self.robot_id)
            if self._duplicates.seen(data["message_id"]):
                self.get_logger().warning(f"Duplicate ignored: {data['message_id']}")
                return
            self._incoming.put(data)
        except Exception as exc:
            self.get_logger().error(f"Invalid MQTT command: {exc}")

    def _process_commands(self) -> None:
        while True:
            try:
                self._pending.append(self._incoming.get_nowait())
            except queue.Empty:
                break

        if self._active_command is not None or not self._pending:
            return

        self._active_command = self._pending.popleft()
        self._active_since = time.monotonic()
        message = String()
        message.data = json.dumps(self._active_command, ensure_ascii=False)
        self._command_pub.publish(message)
        self.get_logger().info(
            f"Published command {self._active_command['message_id']} to ROS agent; "
            f"queued={len(self._pending)}"
        )

    def _active_message_id(self) -> str | None:
        if not self._active_command:
            return None
        value = self._active_command.get("message_id")
        return value if isinstance(value, str) else None

    def _on_ros_answer(self, message: String) -> None:
        try:
            data = normalize_agent_payload(
                message.data,
                expected_robot_id=self.robot_id,
                active_message_id=self._active_message_id(),
                answer=True,
            )
            validate_outgoing(data, self.robot_id, answer=True)
            self._mqtt.publish(
                self.answer_mqtt_topic,
                json.dumps(data, ensure_ascii=False),
                qos=1,
                retain=False,
            )
            self.get_logger().info(
                f"Forwarded final answer for {data['message_id']} to MQTT"
            )

            if data["message_id"] == self._active_message_id():
                self._active_command = None
                self._active_since = None
                self._process_commands()
        except Exception as exc:
            self.get_logger().error(f"Invalid ROS answer: {exc}")

    def _on_ros_status(self, message: String) -> None:
        try:
            data = normalize_agent_payload(
                message.data,
                expected_robot_id=self.robot_id,
                active_message_id=self._active_message_id(),
                answer=False,
            )
            validate_outgoing(data, self.robot_id, answer=False)
            self._mqtt.publish(
                self.status_mqtt_topic,
                json.dumps(data, ensure_ascii=False),
                qos=1,
                retain=False,
            )
        except Exception as exc:
            # A rover can have unrelated legacy publishers on /agent/status.
            # Keep the bridge alive and make the diagnostic explicit.
            self.get_logger().warning(f"Ignored ROS status: {exc}")

    def _check_active_timeout(self) -> None:
        if (
            self._active_command is None
            or self._active_since is None
            or self.agent_command_timeout_sec <= 0
        ):
            return
        elapsed = time.monotonic() - self._active_since
        if elapsed < self.agent_command_timeout_sec:
            return

        message_id = self._active_message_id()
        if message_id:
            payload = {
                "message_id": message_id,
                "robot_id": self.robot_id,
                "status": "error",
                "text": (
                    "Локальный агент не прислал итоговый ответ за "
                    f"{self.agent_command_timeout_sec:g} с."
                ),
            }
            self._mqtt.publish(
                self.answer_mqtt_topic,
                json.dumps(payload, ensure_ascii=False),
                qos=1,
                retain=False,
            )
            self.get_logger().error(f"Agent command timed out: {message_id}")

        self._active_command = None
        self._active_since = None
        self._process_commands()

    def destroy_node(self) -> bool:
        self._mqtt.publish(
            self.availability_mqtt_topic,
            json.dumps({"robot_id": self.robot_id, "online": False}),
            qos=1,
            retain=True,
        )
        self._mqtt.disconnect()
        self._mqtt.loop_stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetTextBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
