> Для сохранённой версии этапа в Раменском используйте актуальную [инструкцию воспроизведения](../../../../docs/REPRODUCIBILITY.md) и [статус реализации](../../../../docs/IMPLEMENTATION_STATUS.md). Приведённые ниже примеры внешних API и прежнего стенда — справка исходного базового пакета. Активная конфигурация соревнований находится в `rover_bringup/config/components/agent.yaml`; MCP через bringup использует порт 8766. Инструмент полной эвакуационной миссии в этом пакете отсутствует.

# Environment-first configuration

In this final package, `FLEET_ROBOT_ID`, server/LLM/MCP and rover interface settings are intended to be exported in `~/.bashrc`. See the root `environment.example.sh`. The prompt presets in `config/` are behavior files, not network/robot identity configuration.

# rover_agent_mcp

`rover_agent_mcp` добавляет к роверу текстового агента и локальный MCP-style JSON-RPC сервер инструментов.

Главная идея:

```text
/agent/text_command std_msgs/String
  -> rover_agent_text_node
  -> OpenAI-compatible LLM API, например OpenRouter
  -> native tool calls или JSON-planner fallback
  -> rover_mcp_server http://127.0.0.1:8765/mcp
  -> ROS 2 services/topics/actions
  -> ровер
```

LLM не получает raw-доступ к ROS CLI, shell, файлам или произвольным topic/service/action. Она может вызывать только tools, описанные в этом пакете.

## Рекомендуемые переменные окружения

Для OpenRouter и других OpenAI-compatible endpoint теперь используются generic `OPENAI_*` переменные:

```bash
export OPENAI_BASE_URL=https://openrouter.ai/api/v1
export OPENAI_MODEL=deepseek/deepseek-v4-flash
export OPENAI_API_KEY=sk-or-...
```

Также поддерживаются старые алиасы:

```bash
OPENROUTER_API_KEY / OPENROUTER_MODEL / OPENROUTER_BASE_URL
SVERK_API_KEY / SVERK_MODEL / SVERK_BASE_URL
```

## Проверка только модели без робота

Отдельных тестовых ROS-ноду/скриптов в пакете нет. Для быстрой проверки LLM используй обычный `curl` к OpenAI-compatible endpoint. Это проверяет только модель и ключ, без `rover_bringup`, MCP и железа.

OpenRouter:

```bash
export OPENAI_BASE_URL=https://openrouter.ai/api/v1
export OPENAI_MODEL=deepseek/deepseek-v4-flash
export OPENAI_API_KEY=sk-or-...

curl -i "$OPENAI_BASE_URL/chat/completions" \
  -H "Authorization: Bearer $OPENAI_API_KEY" \
  -H "Content-Type: application/json" \
  -H "HTTP-Referer: https://sverk-rover.local" \
  -H "X-Title: sverk-rover-agent" \
  -d '{
    "model": "'"$OPENAI_MODEL"'",
    "messages": [{"role": "user", "content": "Ответь одним словом: работает"}],
    "temperature": 0.1
  }'
```

Sverk/LiteLLM:

```bash
export OPENAI_BASE_URL=https://ai.sverk.io/v1
export OPENAI_MODEL=qwen35
export OPENAI_API_KEY=sk-...

curl -i "$OPENAI_BASE_URL/chat/completions" \
  -H "Authorization: Bearer $OPENAI_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "model": "'"$OPENAI_MODEL"'",
    "messages": [{"role": "user", "content": "Ответь одним словом: работает"}],
    "temperature": 0.1
  }'
```

## Запуск агента

Можно запускать агента вообще без LLM-аргументов, если заданы `OPENAI_*`:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py native_tool_mode:=false
```

Важно: `llm_api_key_env` — это **имя переменной окружения**, а не сам ключ и не base URL. Правильно: `llm_api_key_env:=OPENAI_API_KEY`. Неправильно: `llm_api_key_env:=$OPENAI_BASE_URL`.


Сначала подними нужное железо ровера. Для MVP с ездой через `/cmd_vel_test`, лентой и odom:

```bash
ros2 launch rover_bringup robot.launch.py \
  use_imu:=false \
  use_lidar:=false \
  use_camera:=false \
  use_vision:=false \
  use_display:=false \
  use_led_strip:=true \
  use_octoliner:=false \
  use_web:=false \
  use_rosboard:=false \
  use_foxglove:=false \
  use_twist_mux:=true \
  use_sim_time:=false \
  discovery_mode:=configured
```

Для Nav2-команд запускай navigation bringup:

```bash
ros2 launch rover_bringup navigation.launch.py \
  use_camera:=false \
  use_vision:=false \
  use_display:=false \
  use_web:=false \
  use_rosboard:=false \
  use_foxglove:=false \
  use_sim_time:=false \
  discovery_mode:=configured
```

Затем агент:

```bash
export OPENAI_BASE_URL=https://openrouter.ai/api/v1
export OPENAI_MODEL=deepseek/deepseek-v4-flash
export OPENAI_API_KEY=sk-or-...

ros2 launch rover_agent_mcp agent_mcp.launch.py \
  llm_api_key_env:=OPENAI_API_KEY \
  llm_model:=$OPENAI_MODEL \
  llm_base_url:=$OPENAI_BASE_URL \
  native_tool_mode:=auto
```

Если конкретная модель плохо поддерживает native tool calls, можно принудительно использовать JSON planner:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  llm_api_key_env:=OPENAI_API_KEY \
  llm_model:=$OPENAI_MODEL \
  llm_base_url:=$OPENAI_BASE_URL \
  native_tool_mode:=false
```

## ROS topics агента

Версия 0.2 совместима с `fleet_text_bridge_ros2`. Параметр `robot_id` задаёт
идентичность агента:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py robot_id:=rover-01
```

### `/agent/text_command`

Агент принимает новый fleet-envelope:

```json
{"message_id":"...","robot_id":"rover-01","text":"проедь прямо 30 см"}
```

`message_id` и `robot_id` не передаются в LLM. Агент обрабатывает только поле
`text`, а метаданные сохраняет для ответа. Legacy plain text также принимается:

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String \
"{data: 'проедь прямо 30 см'}"
```

### `/agent/status`

JSON со статусом и корреляционными ID:

```json
{"message_id":"...","robot_id":"rover-01","status":"running","text":"Команда получена локальным агентом."}
```

```bash
ros2 topic echo /agent/status
```

### `/agent/answer`

Итоговый JSON-ответ:

```json
{"message_id":"...","robot_id":"rover-01","status":"completed","text":"Готово."}
```

При ошибке `status` равен `error`.

```bash
ros2 topic echo /agent/answer
```

Подробности: `FLEET_PROTOCOL.md`.

## Prompt customization

Агент поддерживает внешний prompt-файл:

```bash
prompt_file:=/path/to/default_system_prompt.md
```

По умолчанию используется:

```text
share/rover_agent_mcp/config/default_system_prompt.md
```

В этом файле можно менять стиль общения: дружелюбный, технический, «как пират» и так далее. Важно: кастомизация должна менять стиль ответа, но не должна искажать имена tools и технические параметры.

Пример запуска:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  llm_api_key_env:=OPENAI_API_KEY \
  llm_model:=$OPENAI_MODEL \
  llm_base_url:=$OPENAI_BASE_URL \
  prompt_file:=/home/pi/prompts/friendly.md \
  native_tool_mode:=auto
```


### Готовые prompt presets

После сборки пакета `.md`-пресеты устанавливаются сюда:

```bash
$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/
```

Доступные варианты:

```text
default_system_prompt.md       основной стиль, коротко и дружелюбно, всегда заканчивает «Бип-буп.»
preset_funny.md                веселый робот с короткими шутками
preset_comedian.md             режим маленького стендап-комика
preset_elegant.md              максимально элегантный и спокойный стиль
preset_swearing_mechanic.md    ворчливый гаражный механик с мягкой грубой лексикой
preset_granny.md               добрая ворчливая бабушка
preset_sarcastic.md            сухой саркастичный робот без токсичности
preset_pirate.md               добродушный робот-пират
preset_strict_engineer.md      строгий инженерный стиль без шуток
```

Пример запуска с пресетом:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  native_tool_mode:=false \
  prompt_file:=$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/preset_funny.md
```

Элегантный режим:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  native_tool_mode:=false \
  prompt_file:=$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/preset_elegant.md
```

Режим бабки:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  native_tool_mode:=false \
  prompt_file:=$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/preset_granny.md
```

Режим ворчливого механика:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  native_tool_mode:=false \
  prompt_file:=$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/preset_swearing_mechanic.md
```

Важно: presets меняют только стиль финального ответа. Имена tools, topic/service/action и численные параметры не должны искажаться.

## MCP endpoint

Локальный MCP-style сервер слушает:

```text
http://127.0.0.1:8765/mcp
```

Поддерживаемые JSON-RPC методы:

```text
initialize
tools/list
tools/call
```

Список tools:

```bash
curl -s http://127.0.0.1:8765/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}' | jq
```

Ручной вызов tool:

```bash
curl -s http://127.0.0.1:8765/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"set_led_preset","arguments":{"preset":"zima_blue"}}}' | jq
```

## Tools

### General

#### `get_available_tools()`

Возвращает категории tools и краткие описания. Используется для вопросов «что ты умеешь?».

#### `wait(duration_s)`

Ждет указанное число секунд. В sequence следующий шаг начинается сразу после окончания ожидания.

### LED strip

#### `set_led_strip(enabled, effect, brightness, color, secondary_color, effect_speed_hz)`

Низкоуровневое управление светодиодной лентой через ROS service:

```text
/led_strip/set_state
```

Эффекты:

```text
fill, blink, blink_fast, fade, wipe, flash, rainbow, rainbow_fill
```

#### `set_led_preset(preset)`

Пресеты:

```text
off, idle, zima_blue, blue, cyan, green, red, white, yellow, purple,
rainbow, thinking, navigation, manual_control, warning, blink_blue,
success, error
```

#### `blink_led_strip(color, times, interval_s, brightness, restore)`

Мигает лентой указанное число раз.

#### `get_led_strip_state()`

Возвращает последнее состояние ленты из `/led_strip/state`.

### Relative mecanum motion

Платформа считается mecanum/omni и может принимать `Twist.linear.y`, поэтому поддерживается боковое и диагональное движение.

Относительное движение использует `/odom` для проверки фактического смещения, а команды отправляет в:

```text
/cmd_vel_test
```

#### `drive_relative(forward_m, left_m, speed_mps, timeout_s)`

Odom-based движение в локальной системе робота:

```text
forward_m > 0  вперед
forward_m < 0  назад
left_m > 0     влево боком
left_m < 0     вправо боком
```

Примеры:

```json
{"forward_m": 0.30, "left_m": 0.0}
{"forward_m": 0.0, "left_m": -0.25}
{"forward_m": 0.30, "left_m": 0.30}
```

#### `turn_relative(angle_deg, angular_speed_degps, timeout_s)`

Odom-based поворот:

```text
+90 = налево
-90 = направо
```

#### `run_motion_sequence(steps, stop_on_error)`

Главный tool для сложных команд.

Поддерживаемые step types:

```text
drive_relative, drive_forward, turn_relative, navigate_to_pose,
set_led_strip, set_led_preset, blink_led_strip, wait, stop_motion
```

Если step `navigate_to_pose` находится внутри sequence, он по умолчанию ждет результат Nav2 action. Следующий шаг начинается сразу после `SUCCEEDED`/`ABORTED`/`CANCELED`, а `timeout_s` используется только как максимальная страховка.

Пример:

```json
{
  "steps": [
    {"type": "drive_relative", "forward_m": 0.30, "left_m": 0.0, "speed_mps": 0.12},
    {"type": "turn_relative", "angle_deg": -90},
    {"type": "drive_relative", "forward_m": 0.0, "left_m": 0.20, "speed_mps": 0.10},
    {"type": "blink_led_strip", "color": "#16B8F3", "times": 3}
  ],
  "stop_on_error": true
}
```

#### Compatibility aliases

```text
drive_forward(distance_m, speed_mps) -> drive_relative(forward_m=distance_m, left_m=0)
run_relative_sequence(steps) -> run_motion_sequence(steps)
```

### Nav2

#### `navigate_to_pose(x, y, yaw_deg, frame_id, wait_until_done, timeout_s)`

Отправляет абсолютную цель в Nav2 action:

```text
/navigate_to_pose
```

Если `wait_until_done=true`, tool возвращается сразу после получения action result. Если Nav2 доехал за 8 секунд, следующий шаг sequence начнется примерно через 8 секунд, а не через весь `timeout_s`.

#### `cancel_navigation()`

Отменяет текущую Nav2-цель.

#### `get_navigation_status()`

Возвращает текущий статус Nav2, последний goal, feedback и pose.

#### `is_navigation_ready()`

Проверяет доступность Nav2 action server и pose.

#### `get_robot_pose()`

Возвращает текущие координаты. Приоритет источников:

```text
/amcl_pose, затем /odom
```

### Diagnostics

#### `get_laser_summary()`

Возвращает краткую сводку по `/scan`: спереди, слева, справа, сзади.

#### `get_system_status()`

Проверяет основные интерфейсы: LED service, Nav2 action, odom, AMCL pose, scan, LED state. Battery status намеренно не включен.

## Примеры команд пользователю

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String \
"{data: 'как тебя зовут и что ты умеешь?'}"
```

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String \
"{data: 'на каких ты сейчас координатах?'}"
```

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String \
"{data: 'проедь прямо 30 сантиметров, потом вправо боком 20 сантиметров и поморгай синим'}"
```

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String \
"{data: 'езжай в точку x 1.2 y -0.4, угол 90 градусов, потом включи зеленую ленту'}"
```
