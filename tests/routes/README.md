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
| `gentle_snake_30_40.json` | Змейка с углами **30–40°** |
| `arc_chain_35.json` | Цепочка дуг по **35°** |
| `boomerang_30_40.json` | «Бумеранг» с углами **30–40°** |
| `circle_polygon_30deg.json` | **Круг** (12×30°), прямые по 12 см |
| `circle_polygon_20deg.json` | **Круг** плавнее (18×20°), прямые по 7 см |
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

### Проверка поворотов при разных углах (один прогон)

Маршрут `sweep_angles_demo.json` — подряд углы **30, 30, 35, 40, 45, 60, 90, 120°** с короткими прямыми.

```bash
HEADING_HOLD_INVERT_STEER=true RASPTANK_DIAG=1 \
  python3 -m tests.test_route tests/routes/sweep_angles_demo.json 2>&1 | tee sweep.log
```

**Нужно:** ровный стол **не меньше ~80×80 см**, робот по центру.

**В логе** у каждого поворота должно быть `reason=angle_reached`; `final_yaw_deg` по модулю близок к `target_deg` / `effective` (допуск несколько градусов). Если часто `timeout` или большой разрыв — IMU/скольжение/настройки creep.

Проверить строки только про повороты:

```bash
grep -E "turn_(left|right)|angle_reached|final_yaw" sweep.log
```
