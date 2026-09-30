# reflector_lighthouse

Локализация по светоотражающим маркерам (идея GOZ-Shuttle).

```text
/scan_light + /odometry/filtered  →  маркеры / поза  →  /light_pose  →  map-EKF
```

## Ноды

| Нода | Launch | Зачем |
|------|--------|--------|
| `marker_mapper_node` | `marker_mapper.launch.py` | Собрать карту маркеров в YAML |
| `lighthouse_node` | `lighthouse.launch.py` | Локализация по YAML → `/light_pose` |

## Сборка

```bash
cd ~/ros2_ws
colcon build --packages-select reflector_lighthouse sllidar_ros2
source install/setup.bash
```

Нужно заранее:
- лидар с `/scan_light` (intensity-фильтр в `sllidar_ros2`);
- map-EKF с `/odometry/filtered` в frame `map`;
- в RViz Fixed Frame = `map`.

---

## Часть A. Собрать и сохранить карту

### 1. Запуск

```bash
ros2 launch reflector_lighthouse marker_mapper.launch.py
```

В логе должна быть строка:

```text
output_file = /home/user/ros2_ws/src/reflector_lighthouse/config/markers.yaml
```

Другой путь:

```bash
ros2 launch reflector_lighthouse marker_mapper.launch.py \
  output_file:=/path/to/markers.yaml
```

### 2. RViz

Добавить **MarkerArray**:

| Topic | Смысл |
|-------|--------|
| `/marker_mapper/detections` | яркие точки текущего скана (оранжевые) |
| `/marker_mapper/markers` | накопленные маркеры (серый = мало хитов, голубой = готов) |

Подпись `3:12` = id `3`, хитов `12`.

Если **detections пустые** — нет ярких точек на `/scan_light` (порог `intensity_min`) или нет одометрии.

Если detections есть, а markers не растут — нет `/odometry/filtered` (нода не знает, куда класть точки в `map`).

### 3. Объезд

Проехать мимо отражателей. Готовый маркер: хитов ≥ `min_observations` (по умолчанию **3**).

### 4. Сохранение

```bash
# только готовые
ros2 service call /marker_mapper/save std_srvs/srv/Trigger

# все треки
ros2 service call /marker_mapper/save_force std_srvs/srv/Trigger

# или топиком:
ros2 topic pub --once /marker_mapper/save_force std_msgs/msg/Empty "{}"
```

Важно: тип **`std_srvs/srv/Trigger`**, не `Triggerer`.

Успех:

```text
success: true
message: Saved N markers to .../config/markers.yaml
```

Проверить файл:

```bash
cat ~/ros2_ws/src/reflector_lighthouse/config/markers.yaml
```

Сброс треков:

```bash
ros2 service call /marker_mapper/clear std_srvs/srv/Trigger
```

При остановке ноды (Ctrl+C / закрытие launch) mapper **сам сохраняет** маркеры в `output_file`
(`save_on_shutdown: true`): сначала готовые, если их нет — все треки.

### Формат YAML

```yaml
markers:
  - id: 0
    x: 1.234
    y: -0.560
    observations: 18
  - id: 1
    x: 3.010
    y: 2.440
    observations: 12
```

Можно править руками. Пример: `config/markers_example.yaml`.

### Созвездия (пары маркеров)

В YAML достаточно списка точек `markers` — **созвездия писать вручную не нужно**.

При старте `lighthouse_node` сам вызывает `build_pairs`:

1. берёт все пары маркеров с расстоянием ≤ `max_pair_dist` (по умолчанию **30 м**);
2. сохраняет каждую пару **в обе стороны** (A→B и B→A);
3. сортирует по длине (как SETDST у GOZ).

Пример: 4 маркера → до 6 уникальных рёбер × 2 направления = **до 12** ориентированных пар.

Опциональный блок `constellations:` в YAML сейчас **не используется** — пары всегда строятся из `markers` через `build_pairs`.

### Если `tracks=0` / файл не создаётся

| Симптом | Что проверить |
|---------|----------------|
| `Nothing to save (tracks=0)` | Нет накопленных маркеров — смотрите таблицу ниже |
| Нет оранжевых detections | `ros2 topic hz /scan_light`, intensity > `intensity_min` |
| Есть detections, нет tracks | `ros2 topic hz /odometry/filtered` — без позы mapper не пишет треки |
| `No markers with >= N hits` | Подождите голубые или вызовите `/marker_mapper/save_force` |
| Сервис не находится / висит | Тип `Trigger` (не `Triggerer`); имя `/marker_mapper/save` |

---

## Часть B. Локализация

### 1. Запуск

```bash
ros2 launch reflector_lighthouse lighthouse.launch.py \
  markers_file:=/home/user/ros2_ws/src/reflector_lighthouse/config/markers.yaml
```

### 2. Выход

`/light_pose` — `geometry_msgs/PoseWithCovarianceStamped`, `frame_id: map`.

Проверка:

```bash
ros2 topic echo /light_pose --once
```

### 3. RViz

| Topic | Тип | Смысл |
|-------|-----|--------|
| `/lighthouse/map_markers` | MarkerArray | маркеры из YAML |
| `/lighthouse/stars` | MarkerArray | текущие детекции |
| `/light_pose` | Pose / PoseWithCovariance | оценка позы |

### 4. EKF

Пакет **не меняет** `ekf.yaml`. Подключите сами, например:

```yaml
poseN: /light_pose
poseN_config: [true, true, false,
               false, false, true,
               false, false, false,
               false, false, false,
               false, false, false,
               false, false, false]
poseN_differential: false
poseN_relative: false
```

Для оценки нужны **≥ 2 маркера** в поле зрения.

---

## Параметры (`config/params.yaml`)

| Параметр | Default | Кто | Смысл |
|----------|---------|-----|--------|
| `scan_topic` | `/scan_light` | оба | скан |
| `odom_topic` | `/odometry/filtered` | оба | поза в `map` |
| `intensity_min` | `190` | оба | порог intensity |
| `cluster_radius_m` | `0.07` | оба | склейка лучей |
| `lidar_x/y/yaw` | `0.01 / 0 / 3.14` | оба | как TF `base_link→laser` в bringup |
| `merge_radius_m` | `0.25` | mapper | слияние треков |
| `min_observations` | `3` | mapper | хитов для «готового» |
| `max_range_m` | `8.0` | mapper | макс. дальность |
| `output_file` | `.../config/markers.yaml` | mapper | куда сохранять |
| `pose_topic` | `/light_pose` | lighthouse | выход |
| `max_dist_error` | `0.08` | lighthouse | допуск длины пары, м |
| `max_xy_error` | `0.5` | lighthouse | допуск к текущей позе, м |
| `min_votes` | `2` | lighthouse | мин. число матчей |
| `pose_variance_xy/yaw` | `0.05` | lighthouse | ковариация |

---

## Команды одной простынёй

```bash
source ~/ros2_ws/install/setup.bash

# A) карта
ros2 launch reflector_lighthouse marker_mapper.launch.py
# ... объезд ...
ros2 service call /marker_mapper_node/save_markers_force std_srvs/srv/Trigger
ros2 service call /marker_mapper/save_force std_srvs/srv/Trigger
cat ~/ros2_ws/src/reflector_lighthouse/config/markers.yaml

# B) локализация
ros2 launch reflector_lighthouse lighthouse.launch.py \
  markers_file:=$HOME/ros2_ws/src/reflector_lighthouse/config/markers.yaml
ros2 topic echo /light_pose --once
```
