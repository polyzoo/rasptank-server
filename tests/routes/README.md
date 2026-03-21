# Примеры маршрутов (JSON)

Все файлы — для `python3 -m tests.test_route <файл>`.

| Файл | Идея |
|------|------|
| `square_40.json` | Замкнутый квадрат 40×40 см |
| `l_shape_50.json` | Буква Г, открытый конец |
| `zigzag_40.json` | Зигзаг, не замкнут |
| `rectangle_open_80x50.json` | Три стороны прямоугольника (П-форма) |
| `triangle_open_50.json` | Две стороны + поворот 120° |
| `lightning_35.json` | Короткая «молния» |
| `rectangle_150x80.json` | Большой прямоугольник (если есть в репо) |
| `obstacle_after_turn.json` | Сценарий с поворотом и препятствием |

## Запуск (как у квадрата с инверсией руления)

```bash
cd ~/rasptank-server
source .venv/bin/activate

HEADING_HOLD_INVERT_STEER=true RASPTANK_DIAG=1 \
  python3 -m tests.test_route tests/routes/l_shape_50.json 2>&1 | tee run_l.log
```

Замени имя файла на нужный маршрут.
