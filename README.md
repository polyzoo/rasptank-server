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

6. Для запуска тестов используйте команду.

```bash
make test
make coverage
```

## Конфигурация

Параметры задаются через `.env` или переменные окружения.

| Переменная                 | По умолчанию | Описание                                    |
|----------------------------|--------------|---------------------------------------------|
| `APP_HOST`                 | 0.0.0.0      | Хост веб-сервера                            |
| `APP_PORT`                 | 8010         | Порт                                        |
| `MIN_OBSTACLE_DISTANCE_CM` | 20           | Дистанция остановки перед препятствием (см) |
| `DECELERATION_DISTANCE_CM` | 10           | Зона торможения (см)                        |
| `BASE_SPEED_PERCENT`       | 55           | Базовая скорость (%)                        |
| `TURN_SPEED_PERCENT`       | 72           | Скорость поворотов (%)                      |

## Структура проекта

```
rasptank-server/
├── src/
│   ├── application/     # Сервисы, модели, протоколы
│   ├── config/          # Настройки
│   ├── infrastructures/ # Моторы, датчики
│   └── presentation/    # API
```
