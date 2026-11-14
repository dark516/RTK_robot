# frob_city_web

Веб-панель оператора Frob City. Интерфейс доступен на `http://<robot-ip>:8080`
и работает без `rosbridge` и стороннего веб-фреймворка.

Панель показывает `/map`, TF-позу и след робота, `/city/planned_route`,
`/city/motion_permit`, состояние миссии и guard, обнаруженные знаки,
диагностику, `/city/cmd_vel_raw`, `/cmd_vel` и размеченный видеопоток.

Управление намеренно ограничено пятью командами: `START`, `PAUSE`,
`RESUME`, `ABORT`, `RESET`. HTTP API не управляет приводом и не публикует
скорость: команды передаются только через сервис `/city/control`.

## Сборка и запуск

```bash
cd /home/pi/Frob_gorod
source /opt/ros/jazzy/setup.zsh
colcon build --packages-select frob_city_web
source install/setup.zsh
ROS_DOMAIN_ID=8 ros2 launch frob_city_web frob_city_web.launch.py
```

Единый безопасный запуск ядра автономности и веб-панели:

```bash
cd /home/pi/Frob_gorod
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch frob_city_web city_robot_test.launch.py
```

Этот launch не поднимает железо, локализацию, Nav2 и детектор без явных
`include_*:=true`. Миссия стартует в `IDLE`, а непрошедший физическую
комиссию `motion_guard` не пропускает команды на привод. Пути к полевому,
событийному, перцепционному и навигационному профилям можно передать через
`field_config`, `event_config`, `perception_config` и `navigation_config`.

Параметры топиков, частоты камеры, HTTP-адреса и пути к конфигурациям
задаются в `config/web.yaml`. Если `/city/control_mission` отсутствует, кнопки
автоматически блокируются.

## Графическая калибровка поля

Страница `http://<robot-ip>:8080/settings/calibration` совмещает фиксированный
пресет полигона 5×5 с Nav2-картой. При наличии свежего `/map` он выбирается
автоматически. Если `/map` отсутствует, интерфейс предлагает YAML-карты из
`/home/pi/Frob_gorod/src/frob_navigation/maps` и установленной копии пакета.

Калибратор создаёт только версионные черновики. Проверка геометрии отдельно
показывает физические параметры, которых ещё не хватает. Активация возможна
только без ошибок и блокеров; предыдущий активный набор сохраняется в backup.
Выбор карты на этой странице не переключает работающий Nav2 map server.

Активные файлы находятся в:

```text
/home/pi/Frob_gorod/calibration/active/calibration.json
/home/pi/Frob_gorod/calibration/active/field.yaml
```

Подробная последовательность приведена в `docs/FIELD_CALIBRATION_GUIDE.md`.

## Проверка

```bash
colcon test --packages-select frob_city_web
colcon test-result --verbose
```

Для безопасной визуальной проверки без связи с реальным роботом можно
запустить `test/fake_dashboard_ros.py` вместе с веб-нодой в отдельном
`ROS_DOMAIN_ID`.
