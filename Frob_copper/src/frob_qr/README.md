# Frob Copper — ArUco маркер трекинг и навигация

## Задача

Робот на базе ROS 2 (Jazzy) на Raspberry Pi должен:

1. Детектить ArUco маркер (4x4 словарь, ID 228) через USB-камеру
2. Выравниваться по маркеру с помощью PID-регулятора, двигаясь вперёд/назад
3. Проезжать заданное расстояние по одометрии, центрируясь по маркеру
4. Публиковать пройденное расстояние в `/dist`

---

## Пакет: `frob_qr`

Расположение: `Frob_copper/src/frob_qr/`

### Состав

```
frob_qr/
├── launch/
│   ├── aruco.launch.py        # запуск детектора + контроллера (для отладки PID)
│   └── navigate.launch.py     # запуск детектора + навигатора (основная задача)
├── frob_qr/
│   ├── __init__.py
│   ├── aruco_detector.py      # нода: захват кадра с камеры, детект ArUco
│   ├── aruco_controller.py    # нода: PID, cmd_vel (устаревшая, для отладки)
│   └── aruco_navigator.py     # нода: конечный автомат вперёд/назад по одометрии
├── resource/
│   └── frob_qr
├── package.xml
├── setup.cfg
└── setup.py
```

---

## Ноды

### 1. `aruco_detector`

**Файл**: `frob_qr/aruco_detector.py`

**Назначение**: открывает камеру напрямую через OpenCV (`cv2.VideoCapture`), детектит ArUco маркер, публикует смещение от центра кадра.

**Топики**:
- `pub` `/aruco/error` (`std_msgs/Float32`) — нормированная ошибка: `-1.0` (маркер слева) ... `+1.0` (справа)
- `pub` `/aruco/detected` (`std_msgs/Bool`) — найден ли целевой маркер в кадре

**Параметры**:
| Параметр | Тип | Дефолт | Описание |
|----------|-----|--------|----------|
| `camera_id` | int | 0 | Индекс камеры |
| `marker_id` | int | 104 | ID целевого ArUco маркера |
| `show_window` | bool | False | Показать OpenCV окно с визуализацией |
| `verbose` | bool | False | Отладочные логи |

**Визуализация** (при `show_window:=True`):
- Зелёная линия по центру кадра
- Обводка всех найденных маркеров с ID
- Красное перекрестие на целевом маркере + синяя линия к центру кадра
- Текст ошибки (цвет от зелёного до красного)

---

### 2. `aruco_controller` (вспомогательная, для отладки PID)

**Файл**: `frob_qr/aruco_controller.py`

Подписывается на `/aruco/error` и `/aruco/detected`, считает PID и шлёт в `cmd_vel`. Используется для ручной настройки коэффициентов. Заменена на `aruco_navigator` для основной задачи.

---

### 3. `aruco_navigator`

**Файл**: `frob_qr/aruco_navigator.py`

**Назначение**: конечный автомат, который едет вперёд N см по одометрии, выравниваясь по ArUco, ждёт, едет назад N см, ждёт, и повторяет цикл.

**Топики**:
- `sub` `/aruco/error` — ошибка центрирования
- `sub` `/aruco/detected` — флаг детекта
- `sub` `/odom` (`nav_msgs/Odometry`) — одометрия
- `pub` `/cmd_vel` (`Twist`) — скорость роботу
- `pub` `/dist` (`Float32`) — пройденное расстояние от старта (в см)

**Конечный автомат**:

```
FORWARD → WAIT_FWD → BACKWARD → WAIT_BWD → FORWARD → ...
```

- **FORWARD**: едет вперёд со скоростью `base_speed`, центрируясь PID(fwd). При достижении `distance` → WAIT_FWD
- **WAIT_FWD**: стоит `wait_duration` секунд → BACKWARD
- **BACKWARD**: едет назад со скоростью `-base_speed`, центрируясь PID(bwd). При достижении `distance` → WAIT_BWD
- **WAIT_BWD**: стоит `wait_duration` секунд → FORWARD

**Параметры**:
| Параметр | Тип | Дефолт | Описание |
|----------|-----|--------|----------|
| `distance` | int | 50 | Дистанция в см |
| `base_speed` | double | 0.3 | Базовая скорость (м/с) |
| `max_angular` | double | 1.5 | Макс. угловая скорость (рад/с) |
| `wait_duration` | int | 2 | Пауза между сегментами (сек) |
| `kp_fwd` | double | 1.5 | P-коэф. движения вперёд |
| `ki_fwd` | double | 0.05 | I-коэф. движения вперёд |
| `kd_fwd` | double | 0.2 | D-коэф. движения вперёд |
| `kp_bwd` | double | -1.5 | P-коэф. движения назад |
| `ki_bwd` | double | -0.05 | I-коэф. движения назад |
| `kd_bwd` | double | -0.2 | D-коэф. движения назад |
| `verbose` | bool | False | Отладочные логи |

---

## Типичные команды запуска

```bash
# Основная задача: 50см вперёд-назад, PID только P
ros2 launch frob_qr navigate.launch.py \
  base_speed:=0.2 distance:=50 \
  kp_fwd:=1.0 ki_fwd:=0.0 kd_fwd:=0.0 \
  kp_bwd:=-1.0 ki_bwd:=0.0 kd_bwd:=0.0 \
  marker_id:=228 camera_id:=0

# С визуализацией и логами
ros2 launch frob_qr navigate.launch.py \
  verbose:=True show_window:=True \
  marker_id:=228 camera_id:=0

# Отладка PID отдельно
ros2 launch frob_qr aruco.launch.py \
  base_speed:=0.2 kp:=1.0 ki:=0.0 kd:=0.0 \
  marker_id:=228 camera_id:=0 verbose:=True

# Пересборка
cd ~/Frob_robot/ros2/src/ros2_ws
colcon build --packages-select frob_qr
```

---

## Известные проблемы и решения

### NumPy 2.x conflict с cv_bridge
**Ошибка**: `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`
**Решение**: `pip install "numpy<2" --force-reinstall`

### Параметры launch передаются не тем типом
**Ошибка**: `InvalidParameterTypeException: expecting type DOUBLE, got INTEGER`
**Решение**: дефолтные значения объявлены как int (50 вместо 50.0), чтение обёрнуто в `float()`

### Словарь ArUco DICT_4X4_100 не содержит ID 228
**Ошибка**: `Markers found: [64] — none matches target ID 228`
**Решение**: используется `DICT_4X4_1000` (ID 0–999)

### PID-коэффициенты не применяются из launch
**Причина**: в launch-файле не были объявлены аргументы `kp`/`ki`/`kd` и не передавались в параметры ноды
**Решение**: все аргументы объявлены в launch и передаются в `parameters=[{...}]`

---

## Что важно для другой модели

- ROS 2 Jazzy на Raspberry Pi (Python 3.12)
- Робот управляется через `/cmd_vel` → Arduino bridge → моторы
- Одометрия приходит в `/odom` от rf2o (лазерная одометрия) или frob_odometry
- Камера — USB, открывается через `cv2.VideoCapture`
- Все ноды в одном пакете `frob_qr`, Python-пакет (ament_python)
- Сборка: `colcon build --packages-select frob_qr`
