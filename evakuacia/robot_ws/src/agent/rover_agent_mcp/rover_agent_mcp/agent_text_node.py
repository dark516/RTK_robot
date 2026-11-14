from __future__ import annotations

import os
from pathlib import Path
import threading
import traceback
from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from rover_agent_mcp.fleet_protocol import (
    CommandContext,
    make_answer_payload,
    make_status_payload,
    parse_command_payload,
)
from rover_agent_mcp.mcp_client import McpJsonRpcClient
from rover_agent_mcp.openrouter_host import DEFAULT_SYSTEM_PROMPT, OpenRouterHost
from rover_agent_mcp.utils import json_dumps


class AgentTextNode(Node):
    """Text agent that understands the fleet transport envelope.

    The node deliberately keeps ``message_id`` and ``robot_id`` out of the
    natural-language command sent to the LLM.  The metadata is preserved in a
    :class:`CommandContext` and copied into every status and final answer.
    """

    def __init__(self) -> None:
        super().__init__('rover_agent_text_node')

        self.declare_parameter('robot_id', os.getenv('FLEET_ROBOT_ID', 'rover-01'))
        self.declare_parameter('text_command_topic', os.getenv('AGENT_TEXT_COMMAND_TOPIC', '/agent/text_command'))
        self.declare_parameter('status_topic', os.getenv('AGENT_STATUS_TOPIC', '/agent/status'))
        self.declare_parameter('answer_topic', os.getenv('AGENT_ANSWER_TOPIC', '/agent/answer'))
        self.declare_parameter('prompt_file', os.getenv('AGENT_PROMPT_FILE', ''))
        self.declare_parameter('mcp_url', os.getenv('MCP_URL', 'http://127.0.0.1:8765/mcp'))
        self.declare_parameter(
            'openrouter_base_url',
            os.getenv(
                'OPENAI_BASE_URL',
                os.getenv(
                    'OPENROUTER_BASE_URL',
                    os.getenv('SVERK_BASE_URL', 'https://openrouter.ai/api/v1'),
                ),
            ),
        )
        self.declare_parameter(
            'llm_base_url',
            os.getenv(
                'OPENAI_BASE_URL',
                os.getenv('OPENROUTER_BASE_URL', os.getenv('SVERK_BASE_URL', '')),
            ),
        )
        self.declare_parameter(
            'openrouter_model',
            os.getenv(
                'OPENAI_MODEL',
                os.getenv('OPENROUTER_MODEL', os.getenv('SVERK_MODEL', '')),
            ),
        )
        self.declare_parameter(
            'llm_model',
            os.getenv(
                'OPENAI_MODEL',
                os.getenv('OPENROUTER_MODEL', os.getenv('SVERK_MODEL', '')),
            ),
        )
        self.declare_parameter('openrouter_api_key_env', 'OPENROUTER_API_KEY')
        self.declare_parameter(
            'llm_api_key_env',
            os.getenv('LLM_API_KEY_ENV', 'OPENAI_API_KEY'),
        )
        self.declare_parameter('llm_api_key_file', '')
        self.declare_parameter(
            'openrouter_http_referer',
            os.getenv('OPENROUTER_HTTP_REFERER', ''),
        )
        self.declare_parameter('app_title', 'sverk-rover-agent')
        self.declare_parameter('timeout_s', float(os.getenv('LLM_TIMEOUT_SEC', '120')))
        self.declare_parameter('max_tool_rounds', int(os.getenv('LLM_MAX_TOOL_ROUNDS', '8')))
        self.declare_parameter(
            'native_tool_mode',
            os.getenv('LLM_NATIVE_TOOL_MODE', 'auto'),
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )

        self.robot_id = str(self.get_parameter('robot_id').value).strip() or 'rover-01'
        self.command_topic = str(self.get_parameter('text_command_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.answer_topic = str(self.get_parameter('answer_topic').value)
        self.prompt_file = str(self.get_parameter('prompt_file').value).strip()
        self.mcp_url = str(self.get_parameter('mcp_url').value)

        self._status_pub = self.create_publisher(String, self.status_topic, 10)
        self._answer_pub = self.create_publisher(String, self.answer_topic, 10)
        self.create_subscription(String, self.command_topic, self._on_text_command, 10)

        self._lock = threading.Lock()
        self._active = False

        self.get_logger().info(
            f'Agent {self.robot_id} listens on {self.command_topic}, '
            f'publishes status to {self.status_topic} and answers to {self.answer_topic}'
        )
        self.get_logger().info(
            'Fleet protocol enabled: transport IDs are not sent to the LLM and '
            'are automatically copied into status/answer envelopes.'
        )
        self.get_logger().info(
            'Set OPENAI_API_KEY/OPENAI_MODEL/OPENAI_BASE_URL before sending '
            'commands. OPENROUTER_* and SVERK_* aliases are still supported.'
        )

    def _publish_json(self, publisher, payload: dict[str, Any]) -> None:  # noqa: ANN001
        publisher.publish(String(data=json_dumps(payload)))

    def _publish_status(
        self,
        context: CommandContext,
        *,
        status: str,
        text: str,
    ) -> None:
        self._publish_json(
            self._status_pub,
            make_status_payload(context, status=status, text=text),
        )

    def _publish_answer(
        self,
        context: CommandContext,
        *,
        status: str,
        text: str,
    ) -> None:
        self._publish_json(
            self._answer_pub,
            make_answer_payload(context, status=status, text=text),
        )

    def _load_system_prompt(self) -> str:
        prompt_path = self.prompt_file
        if not prompt_path:
            return DEFAULT_SYSTEM_PROMPT
        try:
            path = Path(prompt_path).expanduser()
            if not path.is_file():
                self.get_logger().warning(
                    f'prompt_file does not exist: {path}; using built-in default prompt'
                )
                return DEFAULT_SYSTEM_PROMPT
            custom_prompt = path.read_text(encoding='utf-8').strip()
            if not custom_prompt:
                self.get_logger().warning(
                    f'prompt_file is empty: {path}; using built-in default prompt'
                )
                return DEFAULT_SYSTEM_PROMPT
            return (
                DEFAULT_SYSTEM_PROMPT
                + '\n\n# Пользовательская кастомизация поведения\n'
                + custom_prompt
            )
        except Exception as exc:
            self.get_logger().warning(
                f'Failed to read prompt_file {prompt_path}: {exc}; '
                'using built-in default prompt'
            )
            return DEFAULT_SYSTEM_PROMPT

    @staticmethod
    def _mode_to_string(value: Any) -> str:
        if isinstance(value, bool):
            return 'true' if value else 'false'
        text = str(value or 'auto').strip().lower()
        if text in {'1', 'yes', 'y', 'on'}:
            return 'true'
        if text in {'0', 'no', 'n', 'off'}:
            return 'false'
        return text if text in {'auto', 'true', 'false'} else 'auto'

    def _make_host(self) -> OpenRouterHost:
        # New generic llm_* parameters are preferred. openrouter_* names are kept
        # for backward compatibility with the first version of this package.
        llm_key_env = str(self.get_parameter('llm_api_key_env').value).strip()
        openrouter_key_env = str(
            self.get_parameter('openrouter_api_key_env').value
        ).strip()

        api_key = self._resolve_api_key(
            str(self.get_parameter('llm_api_key_file').value).strip(),
            (
            llm_key_env,
            openrouter_key_env,
            'OPENAI_API_KEY',
            'OPENROUTER_API_KEY',
            'SVERK_API_KEY',
            ),
        )

        llm_model = str(self.get_parameter('llm_model').value).strip()
        openrouter_model = str(self.get_parameter('openrouter_model').value).strip()
        model = (
            llm_model
            or openrouter_model
            or os.getenv('OPENAI_MODEL', '')
            or os.getenv('OPENROUTER_MODEL', '')
            or os.getenv('SVERK_MODEL', '')
        )

        llm_base_url = str(self.get_parameter('llm_base_url').value).strip()
        openrouter_base_url = str(
            self.get_parameter('openrouter_base_url').value
        ).strip()
        base_url = (
            llm_base_url
            or os.getenv('OPENAI_BASE_URL', '')
            or os.getenv('OPENROUTER_BASE_URL', '')
            or os.getenv('SVERK_BASE_URL', '')
            or openrouter_base_url
            or 'https://openrouter.ai/api/v1'
        )

        return OpenRouterHost(
            api_key=api_key,
            model=model,
            base_url=base_url,
            timeout_s=float(self.get_parameter('timeout_s').value),
            max_tool_rounds=int(self.get_parameter('max_tool_rounds').value),
            http_referer=str(self.get_parameter('openrouter_http_referer').value),
            app_title=str(self.get_parameter('app_title').value),
            system_prompt=self._load_system_prompt(),
            native_tool_mode=self._mode_to_string(
                self.get_parameter('native_tool_mode').value
            ),
        )

    @staticmethod
    def _resolve_api_key(key_file: str, env_names: tuple[str, ...]) -> str:
        """Read the local gateway key without committing it to package YAML."""
        if key_file:
            path = Path(key_file).expanduser()
            try:
                value = path.read_text(encoding='utf-8').strip()
            except OSError:
                value = ''
            if value:
                return value
        for env_name in env_names:
            if env_name and os.getenv(env_name):
                return os.getenv(env_name, '')
        return ''

    @staticmethod
    def _search_result_reply(result: dict[str, Any]) -> str | None:
        """Turn the one allowed search tool result into a truthful human report."""
        for call in result.get('tool_results') or []:
            if call.get('name') != 'search_static_obstacle':
                continue
            arguments = call.get('arguments') or {}
            outcome = call.get('result') or {}
            area = str(arguments.get('static_obstacle') or 'выбранной зоны')
            target = str(arguments.get('target_obstacle') or 'any')
            target_names = {
                'traffic_accident': 'ДТП',
                'fuel_truck': 'топливозаправщик',
                'fallen_tree': 'упавшее дерево',
                'building_debris': 'обломки здания',
                'any': 'динамическое препятствие',
            }
            lines = [
                f'Понял задачу: пострадавшего нужно искать около зоны «{area}».',
                f'Запустил объезд этой зоны и поиск QR рядом с целью: {target_names.get(target, target)}.',
            ]
            inspection = outcome.get('inspection') or {}
            obstacle = inspection.get('target_obstacle_cell') or {}
            if obstacle.get('class'):
                lines.append(
                    f'VLM подтвердил цель: {target_names.get(str(obstacle["class"]), obstacle["class"])}. '
                    'Проверил доступные стороны препятствия, не заезжая в его клетку.'
                )
            qr_codes = outcome.get('qr_codes') or inspection.get('qr_codes') or []
            if qr_codes:
                lines.append(f'QR с данными найден: {", ".join(str(code) for code in qr_codes)}.')
                lines.append(f'Подождал у QR {float(outcome.get("waited_at_qr_sec", 5.0)):g} секунд.')
            else:
                lines.append('QR с декодированным текстом не найден.')
            returned = outcome.get('return_to_start') or {}
            if returned.get('success'):
                lines.append('Вернулся в точку «старт».')
            elif qr_codes:
                lines.append('QR найден, но вернуться в точку «старт» не удалось.')
            else:
                detail = str(outcome.get('error') or inspection.get('message') or '').strip()
                if detail:
                    lines.append(f'Поиск завершился без результата: {detail}')
            return '\n'.join(lines)
        return None

    def _on_text_command(self, msg: String) -> None:
        try:
            context = parse_command_payload(msg.data, self.robot_id)
        except ValueError as exc:
            self.get_logger().warning(f'Ignoring invalid command: {exc}')
            return

        if (
            context.source_robot_id
            and context.source_robot_id != self.robot_id
        ):
            # The bridge is the routing authority.  Do not leak or use the
            # incoming robot_id in the LLM prompt; answer under the configured ID.
            self.get_logger().warning(
                f'Incoming envelope says robot_id={context.source_robot_id!r}; '
                f'configured robot_id={self.robot_id!r}. The configured ID will '
                'be used in replies.'
            )

        with self._lock:
            if self._active:
                text = 'Агент всё ещё выполняет предыдущую команду.'
                self._publish_status(context, status='error', text=text)
                self._publish_answer(context, status='error', text=text)
                return
            self._active = True

        worker = threading.Thread(
            target=self._run_command_thread,
            args=(context,),
            daemon=True,
        )
        worker.start()

    def _run_command_thread(self, context: CommandContext) -> None:
        command = context.text
        try:
            self.get_logger().info(
                f'Received agent command {context.message_id}: {command}'
            )
            self._publish_status(
                context,
                status='running',
                text='Команда получена локальным агентом.',
            )

            mcp = McpJsonRpcClient(
                self.mcp_url,
                timeout_s=float(self.get_parameter('timeout_s').value),
            )
            host = self._make_host()
            self._publish_status(
                context,
                status='running',
                text=f'Агент обрабатывает команду моделью {host.model}.',
            )

            # Only the natural-language command is sent to the LLM.  Correlation
            # metadata remains in ``context`` and is restored below.
            result = host.run_command(command, mcp)
            self.get_logger().info(f'Agent result: {json_dumps(result)}')

            reply = self._search_result_reply(result) or str(result.get('reply') or 'Готово.').strip()
            self._publish_status(
                context,
                status='running',
                text='Команда выполнена, формирую итоговый ответ.',
            )
            self._publish_answer(
                context,
                status='completed',
                text=reply,
            )
        except Exception as exc:  # pragma: no cover - runtime guard for demos
            self.get_logger().error(
                f'Agent command failed: {exc}\n{traceback.format_exc()}'
            )
            error_text = f'Ошибка агента: {exc}'
            self._publish_status(context, status='error', text=error_text)
            self._publish_answer(context, status='error', text=error_text)
        finally:
            with self._lock:
                self._active = False


def main() -> None:
    rclpy.init()
    node = AgentTextNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
