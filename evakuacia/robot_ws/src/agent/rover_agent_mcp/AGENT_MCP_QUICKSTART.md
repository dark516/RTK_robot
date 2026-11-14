# Agent MCP quickstart

Подробная документация агента теперь лежит здесь:

```text
src/rover_agent_mcp/README.md
```

Короткий запуск через Sverk AI:

```bash
cd ~/sverk_rover_sverk_ai_agent_mcp_fixed/sverk_rover-main
source /opt/ros/jazzy/setup.bash
source install/setup.bash

export SVERK_API_KEY="your_sk_key_here"
export SVERK_MODEL="qwen35"
export SVERK_BASE_URL="https://ai.sverk.io/v1"

ros2 launch rover_agent_mcp agent_mcp.launch.py \
  llm_api_key_env:=SVERK_API_KEY \
  llm_model:=$SVERK_MODEL \
  llm_base_url:=$SVERK_BASE_URL \
  native_tool_mode:=auto
```

Ответы для пользователя:

```bash
ros2 topic echo /agent/answer
```

Технический статус:

```bash
ros2 topic echo /agent/status
```

Команда:

```bash
ros2 topic pub --once /agent/text_command std_msgs/msg/String \
"{data: 'проедь прямо 30 см, поверни направо на 90 градусов и поморгай лентой'}"
```

Кастомный prompt:

```bash
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  llm_api_key_env:=SVERK_API_KEY \
  llm_model:=qwen35 \
  llm_base_url:=https://ai.sverk.io/v1 \
  prompt_file:=/home/pi/prompts/friendly.md
```

## Prompt presets

Основной prompt теперь заканчивает финальные ответы фразой «Бип-буп.».

Пресеты после сборки лежат тут:

```bash
$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/
```

Примеры:

```bash
# веселый режим
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  native_tool_mode:=false \
  prompt_file:=$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/preset_funny.md

# элегантный режим
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  native_tool_mode:=false \
  prompt_file:=$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/preset_elegant.md

# ворчливый механик
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  native_tool_mode:=false \
  prompt_file:=$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/preset_swearing_mechanic.md

# бабка
ros2 launch rover_agent_mcp agent_mcp.launch.py \
  native_tool_mode:=false \
  prompt_file:=$(ros2 pkg prefix rover_agent_mcp)/share/rover_agent_mcp/config/preset_granny.md
```



Единые переменные окружения для Sverk или OpenRouter:

```bash
export OPENAI_BASE_URL="https://ai.sverk.io/v1"              # Sverk
export OPENAI_MODEL="qwen35"
export OPENAI_API_KEY="..."
```

или:

```bash
export OPENAI_BASE_URL="https://openrouter.ai/api/v1"       # OpenRouter
export OPENAI_MODEL="deepseek/deepseek-v4-flash"
export OPENAI_API_KEY="..."
```
