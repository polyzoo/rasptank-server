# API для интерфейса выбора точки

## Назначение

Этот документ описывает только те endpoint'ы, которые нужны фронтенду для сценария:
- пользователь видит положение машинки на плоскости;
- пользователь выбирает одну целевую точку;
- машинка едет к этой точке;
- фактическая траектория отображается как след;
- при обнаружении неизвестного препятствия маршрут перестраивается;
- если цель недостижима, интерфейс получает это состояние;
- пользователь может отменить движение.

В документе специально нет лишних endpoint'ов:
- нет `L1`;
- нет ручного `cmd_vel`;
- нет маршрута из нескольких точек;
- нет ручного `step`.

## Что нужно фронтенду

Для этого интерфейса нужны только:
- сброс стартового состояния машинки;
- запуск движения к одной точке;
- отмена движения;
- получение текущей позы машинки;
- получение текущего состояния верхнего уровня;
- получение обновлений по websocket.

## Нужные endpoint'ы

### 1. Сбросить стартовое состояние

```text
POST /v1/l2/reset-state
```

Нужен перед новым запуском, если в интерфейсе стартовая точка задаётся явно.

Тело запроса:

```json
{
  "x_cm": 0.0,
  "y_cm": 0.0,
  "heading_deg": 0.0,
  "linear_speed_cm_per_sec": 0.0,
  "angular_speed_deg_per_sec": 0.0
}
```

Ответ:

```json
{
  "x_cm": 0.0,
  "y_cm": 0.0,
  "heading_deg": 0.0,
  "linear_speed_cm_per_sec": 0.0,
  "angular_speed_deg_per_sec": 0.0,
  "left_percent": 0.0,
  "right_percent": 0.0,
  "distance_cm": null
}
```

### 2. Отправить выбранную пользователем целевую точку

```text
POST /v1/l3/goal
```

Это основной endpoint запуска движения к точке.

Если препятствия заранее неизвестны, фронтенд передаёт пустой список:

```json
{
  "target": {
    "x_cm": 120.0,
    "y_cm": 35.0
  },
  "obstacles": []
}
```

Ответ:

```json
{
  "status": "tracking",
  "mode": "point",
  "planner_status": "planned",
  "target_x_cm": 120.0,
  "target_y_cm": 35.0,
  "active_point_index": 0,
  "total_points": 1,
  "distance_error_cm": null,
  "heading_error_deg": null,
  "target_heading_deg": null,
  "linear_speed_cm_per_sec": 0.0,
  "angular_speed_deg_per_sec": 0.0
}
```

Важно:
- если во время движения будет обнаружено неизвестное препятствие, `L3` сам перестроит путь;
- в этом случае дальнейшие обновления придут через `GET /v1/l3/state` или `WS /v1/l3/ws`.

### 3. Отменить текущее движение

```text
POST /v1/l3/cancel
```

Нужен для кнопки `Стоп` или `Отмена`.

Ответ:

```json
{
  "status": "cancelled",
  "mode": "idle",
  "planner_status": "idle",
  "target_x_cm": null,
  "target_y_cm": null,
  "active_point_index": null,
  "total_points": 0,
  "distance_error_cm": null,
  "heading_error_deg": null,
  "target_heading_deg": null,
  "linear_speed_cm_per_sec": 0.0,
  "angular_speed_deg_per_sec": 0.0
}
```

### 4. Получить текущую позу машинки

```text
GET /v1/l2/state
```

Нужен для:
- первичной загрузки состояния;
- отображения текущего положения машинки;
- построения следа движения.

Ответ:

```json
{
  "x_cm": 24.5,
  "y_cm": 11.2,
  "heading_deg": 18.0,
  "linear_speed_cm_per_sec": 12.0,
  "angular_speed_deg_per_sec": 6.0,
  "left_percent": 34.0,
  "right_percent": 42.0,
  "distance_cm": 67.0
}
```

Для отображения следа фронту нужны прежде всего:
- `x_cm`
- `y_cm`
- `heading_deg`

### 5. Получить текущее состояние движения к цели

```text
GET /v1/l3/state
```

Нужен для:
- первичной загрузки состояния миссии;
- восстановления интерфейса после перезагрузки страницы;
- отображения текущего статуса движения.

Ответ:

```json
{
  "status": "tracking",
  "mode": "point",
  "planner_status": "planned",
  "target_x_cm": 120.0,
  "target_y_cm": 35.0,
  "active_point_index": 0,
  "total_points": 1,
  "distance_error_cm": 48.6,
  "heading_error_deg": 7.5,
  "target_heading_deg": 20.0,
  "linear_speed_cm_per_sec": 15.0,
  "angular_speed_deg_per_sec": 12.0,
  "detected_obstacle_x_cm": null,
  "detected_obstacle_y_cm": null,
  "detected_obstacle_radius_cm": null,
  "detected_obstacle_kind": null
}
```

## Нужные websocket'ы

### 1. Поток фактической позы машинки

```text
WS /v1/l2/ws
```

Этот websocket нужен для:
- анимации положения машинки;
- построения следа фактического движения;
- обновления направления машинки в реальном времени.

Формат сообщения:

```json
{
  "x_cm": 24.5,
  "y_cm": 11.2,
  "heading_deg": 18.0,
  "linear_speed_cm_per_sec": 12.0,
  "angular_speed_deg_per_sec": 6.0,
  "left_percent": 34.0,
  "right_percent": 42.0,
  "distance_cm": 67.0
}
```

Для следа фронтенд должен использовать последовательность точек:

```text
(x_cm, y_cm)
```

### 2. Поток состояния движения к цели

```text
WS /v1/l3/ws
```

Этот websocket нужен для:
- статуса `idle / tracking / reached / blocked / unreachable / cancelled`;
- отображения активной цели;
- отображения факта автоматического перестроения пути при неизвестном препятствии.

Формат сообщения:

```json
{
  "status": "tracking",
  "mode": "point",
  "planner_status": "replanned_dynamic",
  "target_x_cm": 83.0,
  "target_y_cm": 21.0,
  "active_point_index": 0,
  "total_points": 3,
  "distance_error_cm": 26.4,
  "heading_error_deg": 11.0,
  "target_heading_deg": 16.0,
  "linear_speed_cm_per_sec": 10.0,
  "angular_speed_deg_per_sec": 18.0,
  "detected_obstacle_x_cm": 62.0,
  "detected_obstacle_y_cm": 18.0,
  "detected_obstacle_radius_cm": 8.0,
  "detected_obstacle_kind": "dynamic"
}
```

Эти поля нужны фронтенду для отрисовки точки ошибки или найденного препятствия:
- `detected_obstacle_x_cm`
- `detected_obstacle_y_cm`
- `detected_obstacle_radius_cm`
- `detected_obstacle_kind`

Если новое неизвестное препятствие ещё не обнаружено, эти поля равны `null`.

## Какие статусы должен понимать фронтенд

Поле:

```text
status
```

Может принимать значения:
- `idle` — активной цели нет;
- `tracking` — машинка движется к цели;
- `reached` — цель достигнута;
- `blocked` — движение остановлено перед препятствием;
- `unreachable` — цель недостижима в допустимом коридоре;
- `cancelled` — движение отменено пользователем.

Поле:

```text
planner_status
```

Может принимать значения:
- `idle` — планировщик не используется;
- `planned` — путь построен;
- `replanned_dynamic` — путь перестроен после обнаружения неизвестного препятствия;
- `impossible` — допустимый путь не найден.

Поле:

```text
detected_obstacle_kind
```

Сейчас может принимать:
- `dynamic` — препятствие впервые обнаружено датчиком во время движения;
- `null` — нового динамического препятствия пока нет.

## Минимальный рабочий сценарий для фронтенда

1. При открытии экрана запросить:
   - `GET /v1/l2/state`
   - `GET /v1/l3/state`
2. Открыть websocket'ы:
   - `WS /v1/l2/ws`
   - `WS /v1/l3/ws`
3. Перед запуском новой миссии при необходимости вызвать:
   - `POST /v1/l2/reset-state`
4. При выборе пользователем точки вызвать:
   - `POST /v1/l3/goal`
5. Для кнопки остановки использовать:
   - `POST /v1/l3/cancel`

## Какие endpoint'ы фронту не нужны для этого сценария

Для интерфейса выбора одной точки не нужны:
- `POST /v1/l1/tracks`
- `POST /v1/l1/stop`
- `GET /v1/l1/state`
- `POST /v1/l2/cmd-vel`
- `POST /v1/l2/stop`
- `POST /v1/l3/route`
- `POST /v1/l3/step`
