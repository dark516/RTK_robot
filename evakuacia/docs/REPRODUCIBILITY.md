# Сборка и воспроизведение компонентов

## Окружение и раздельные workspace

Ubuntu 24.04, ROS 2 Jazzy, Python 3.12. Требуются colcon, rosdep, OpenCV, NumPy и PyYAML. GGUF-модели и аппаратные калибровки не включены. Полный исполнитель миссии в снимке отсутствует: [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md).

`robot_ws` и `ai_ws` содержат разные пакеты с именем `rover_vision`. **Два отдельных терминала / машины обязательны.** Не выполняйте source обоих install в одном shell. Сборка ниже задаёт `--base-paths src`, чтобы colcon не собирал другие решения из родительского репозитория.

Для Python ROS-пакетов `ament_python` предоставляется установкой ROS/colcon; `ament_pytest` — старое тестовое имя. Они пропускаются в команде rosdep ниже. В текущей базе rosdep также отсутствует правило `python3-spidev`; на роботе установите пакет явно:

```bash
sudo apt install python3-spidev python3-paho-mqtt
```

`spidev` используется подсветкой, MQTT — необязательным fleet bridge. При программной проверке на ноутбуке эти аппаратные компоненты не запускались.

На каждой машине сначала загрузите только ROS, установите зависимости и соберите соответствующий workspace:

```bash
source /opt/ros/jazzy/setup.bash
# Выполнить в evakuacia/robot_ws ИЛИ evakuacia/ai_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --skip-keys "ament_python ament_pytest python3-spidev"
python3 /usr/bin/colcon build --base-paths src --symlink-install
source install/local_setup.bash
export ROS_DOMAIN_ID=77
```

Если Python-зависимости установлены в `.venv`, используйте `python /usr/bin/colcon`, где `python` принадлежит окружению, и проверьте shebang установленного узла. Простая активация venv после сборки системным Python не меняет интерпретатор ROS-узлов.

## Ноутбук: LLM

В сохранённой конфигурации использовался llama.cpp с Vulkan. Соберите совместимый `llama-server` по [инструкции runtime](https://github.com/ggml-org/llama.cpp/blob/master/docs/build.md). Скопируйте исполняемый файл в `~/.local/bin/llama-server` либо передайте фактический путь аргументом `llama_server`. Точная ревизия исторической сборки runtime не зафиксирована; исходники runtime не входят в репозиторий.

LLM по умолчанию: Qwen3.5-9B Q4_K_M. Реестр `local_llm_gateway.py` также содержит 2B Q4_K_M и 35B-A3B Q3_K_S. Загрузчик использует сохранённые URL и проверяет размер, но не криптографический digest. Выполнение загрузчика скачивает гигабайты; тесты и сборка его не вызывают.

```bash
# Из evakuacia/ai_ws; установлен aria2
bash src/rover_vision/scripts/download_llm_models.sh 9b
# При необходимости: 2b, 35b-a3b или явно all
```

Каталог по умолчанию — `~/.local/share/rtk-evacuation/models/llm`; он переопределяется `MODELS_ROOT` при загрузке и `models_dir` при запуске.

Создайте общий ключ и передайте тот же файл на робота по доверенному каналу:

```bash
mkdir -p ~/.config/sverk-rover
umask 077
python3 -c 'import secrets; print("sk-local-" + secrets.token_urlsafe(32))' \
  > ~/.config/sverk-rover/local_llm_api_key
```

Сначала можно запустить UI без модели:

```bash
ros2 launch rover_vision local_llm_gateway.launch.py autostart:=false
```

Затем с существующими runtime и весами:

```bash
ros2 launch rover_vision local_llm_gateway.launch.py \
  llama_server:="$HOME/.local/bin/llama-server" \
  models_dir:="$HOME/.local/share/rtk-evacuation/models/llm" \
  default_model:=qwen3.5-9b device:=Vulkan0
```

UI: `https://LAPTOP_IP:8082`, API: `http://LAPTOP_IP:8083/v1`. Backend llama.cpp — loopback `:8092`. Сертификат локальный, самоподписанный. HTTP API и диагностические UI предназначены для доверенной локальной сети; их не следует публиковать в Интернет. Проверка API без робота:

```bash
KEY=$(<~/.config/sverk-rover/local_llm_api_key)
curl http://127.0.0.1:8083/v1/chat/completions \
  -H "Authorization: Bearer $KEY" -H 'Content-Type: application/json' \
  -d '{"model":"qwen3.5-9b","messages":[{"role":"user","content":"Привет"}]}'
```

## Ноутбук: VLM

Веса: `Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf` и `mmproj-Qwen2.5-VL-3B-Instruct-Q8_0.gguf` из [репозитория ggml-org](https://huggingface.co/ggml-org/Qwen2.5-VL-3B-Instruct-GGUF). Скачайте соответствующую пару в `~/.local/share/rtk-evacuation/models/vlm`. В этом восстановлении загрузка и инференс GGUF не повторялись.

```bash
"$HOME/.local/bin/llama-server" \
  --model "$HOME/.local/share/rtk-evacuation/models/vlm/Qwen2.5-VL-3B-Instruct-Q4_K_M.gguf" \
  --mmproj "$HOME/.local/share/rtk-evacuation/models/vlm/mmproj-Qwen2.5-VL-3B-Instruct-Q8_0.gguf" \
  --host 127.0.0.1 --port 8091 --device Vulkan0 --gpu-layers 99 \
  --ctx-size 4096 --image-min-tokens 1024
```

В другом **AI-терминале**:

```bash
ros2 launch rover_vision vlm_diagnostic.launch.py \
  robot_url:='http://ROBOT_IP:8091/stream?mode=live' \
  vlm_server_url:='http://127.0.0.1:8091/v1/chat/completions' mode:=service
ros2 service call /vlm/analyze_frame std_srvs/srv/Trigger '{}'
```

`ROBOT_IP` заменяется реальным адресом. UI VLM: `https://LAPTOP_IP:8081`. Порт камеры 8091 находится на роботе, backend 8091 — на ноутбуке. Результат сообщает класс события и наличие QR, но не декодированную строку QR.

## Робот: аппаратный запуск

В `rover_bringup/config/components/agent.yaml` замените IP в `llm_base_url`. Файл ключа читается из `~/.config/sverk-rover/local_llm_api_key`; права `0600`. Аппаратная конфигурация находится в `rover_v1.yaml` и `components/`. Проверьте геометрию, тип шасси, порядок моторов и устройства по своей платформе. Сохранённый профиль не является калибровкой другого робота.

Карта не включена; подставьте существующий путь:

```bash
ros2 launch rover_bringup robot.launch.py profile:=navigation \
  map:=/absolute/path/to/measured-map.yaml \
  use_agent:=true use_fleet_bridge:=false use_vision:=false
```

Профиль `navigation` включает аппаратные узлы, Nav2, камеру и веб-панель. `use_vision:=false` отключает старый аппаратный COCO-детектор, веса которого не включены. LLM/VLM/CV при таком развёртывании работают в AI workspace ноутбука. Не запускайте одновременно второй отдельный MCP-сервер на том же порту.

## CV: диагностика без обещания полной автономной миссии

```bash
ros2 launch rover_vision path_clearance.launch.py \
  camera_url:='http://ROBOT_IP:8091/frame?mode=live'
```

UI: `http://LAPTOP_IP:8080`. Задайте дорожный ROI и сохраните несколько заведомо свободных кадров. Код содержит два профиля (`normal`, `bridge`) и список поз подходов к мостам. Он требует свежий `/rover/grid_pose`; **издателя этого топика в снимке нет**.

Только для неподвижной диагностической проверки можно вручную задавать положение:

```bash
ros2 topic pub -r 2 /rover/grid_pose std_msgs/msg/String \
  "{data: '{\"cell\":{\"row\":0,\"column\":0},\"view_direction\":{\"x\":1.0,\"y\":0.0}}'}"
```

Это искусственная поза для проверки UI и выбора профиля; её нельзя использовать как источник локализации при движении. Для автономной работы требуется калиброванное преобразование map→клетка. Без свежей позы или эталона результат остаётся `unknown`.

## Проверки

В AI workspace после сборки:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q --import-mode=importlib src/rover_vision/test
```

В robot workspace после сборки, в другом терминале:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q --import-mode=importlib \
  src/agent/rover_agent_mcp/test src/peripherals/rover_lidar_filter/test
```

Архивные systemd-файлы и скрипты могут содержать прежние каталоги (`/home/pi/sverk_rover`, `/home/dark516/...`). Они не устанавливаются автоматически и требуют отдельной адаптации. Автономный заезд, возврат и QR-декодирование не заявляются воспроизведёнными этими командами.
