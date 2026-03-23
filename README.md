# RaspTank Server

Сервер управления машинкой RaspTank на Raspberry Pi: автономное движение по маршруту, плавное
торможение перед препятствиями, REST API.

## Требования

- Python 3.9+
- Raspberry Pi с Adeept Robot HAT V3.1 (PCA9685, HC-SR04)

## Быстрый старт

1. Установите системные зависимости.

```bash
sudo apt-get update
sudo apt-get install -y python3-gpiozero python3-pigpio python3-rpi.gpio python3-lgpio
```

2. Клонируйте репозиторий и перейдите в каталог проекта.

```bash
git clone https://github.com/polyzoo/rasptank-server.git
cd rasptank-server
```

3. Создайте виртуальное окружение и установите зависимости.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

4. Запустите веб-сервер.

```bash
uvicorn src.main:app --host 0.0.0.0 --port 8010
```

5. Откройте `http://localhost:8010/docs` для Swagger UI.

## API

| Метод | Endpoint          | Описание              |
|-------|-------------------|-----------------------|
| POST  | `/v1/drive/route` | Выполнение маршрута   |
| POST  | `/v1/drive/stop`  | Немедленная остановка |

### Пример

```bash
curl -X POST http://localhost:8010/v1/drive/route \
  -H "Content-Type: application/json" \
  -d '{"segments": [
    {"action": "forward", "distance_cm": 100},
    {"action": "turn_left", "angle_deg": 90},
    {"action": "forward", "distance_cm": 100}
  ]}'
```

## Тестирование

**Цель:** Проверить маршрутное движение, калибровку скорости и экспериментальные сценарии.

**Текущее состояние набора:**
- `route` - выполнение маршрута из JSON.
- `calibrate-speed` - калибровка максимальной скорости.
- `static-velocity` - эксперимент `v(U)`.
- `time-constant` - эксперимент постоянной времени.
- `kl-kr-turn` - эксперимент разворота на месте.

**Базовые команды:**
```bash
python -m tests route tests/routes/square_40.json
python -m tests calibrate-speed 3
python -m tests static-velocity 50 5
python -m tests time-constant 60 5
python -m tests kl-kr-turn
```

**Файлы маршрутов:**
- [`square_40.json`](tests/routes/square_40.json)
- [`straight_5m.json`](tests/routes/straight_5m.json)
- [`turn_in_place_90.json`](tests/routes/turn_in_place_90.json)
- [`accel_probe_5m.json`](tests/routes/accel_probe_5m.json)

## Конфигурация

Параметры задаются через `.env` или переменные окружения.

### Основные

| Переменная                 | По умолчанию | Описание                                    |
|----------------------------|--------------|---------------------------------------------|
| `APP_HOST`                 | 0.0.0.0      | Хост веб-сервера                            |
| `APP_PORT`                 | 8010         | Порт                                        |
| `MIN_OBSTACLE_DISTANCE_CM` | 20           | Дистанция остановки перед препятствием (см) |
| `DECELERATION_DISTANCE_CM` | 10           | Зона торможения (см)                        |
| `BASE_SPEED_PERCENT`       | 55           | Базовая скорость (%)                        |
| `TURN_SPEED_PERCENT`       | 72           | Скорость поворотов (%)                      |

### Калибровка

| Переменная                 | По умолчанию | Описание                                 |
|----------------------------|--------------|------------------------------------------|
| `TL_LEFT_OFFSET`           | 0            | Смещение левого мотора (прямолинейность) |
| `TL_RIGHT_OFFSET`          | 0            | Смещение правого мотора                  |
| `MAX_SPEED_CM_PER_SEC`     | 28           | Скорость при 100% для оценки пути        |

## Структура проекта

```
rasptank-server/
├── src/
│   ├── application/     # Сервисы, модели, протоколы
│   ├── config/          # Настройки
│   ├── infrastructures/ # Моторы, датчик
│   └── presentation/    # API
├── tests/               # Тесты и калибровка
│   └── routes/          # JSON-маршруты для тестов
```
