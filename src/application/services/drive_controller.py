import logging
import time
from typing import Optional, final
from threading import Thread, Event

from src.application.protocols import (
    DriveControllerProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)


logger: logging.Logger = logging.getLogger(__name__)


@final
class DriveController(DriveControllerProtocol):
    """Контроллер для управления движением машинки."""

    # Примерная линейная скорость машинки при 100% мощности (см/с)
    MAX_SPEED_CM_PER_SEC: float = 30.0

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        min_obstacle_distance_cm: float = 20.0,
        deceleration_distance_cm: float = 50.0,
        base_speed_percent: int = 60,
        update_interval_sec: float = 0.1,
    ) -> None:
        """Инициализация контроллера движения.

        Контроллер использует `motor_controller` для низкоуровневого управления моторами
        и `ultrasonic_sensor` для получения расстояния до препятствий.

        Порог `min_obstacle_distance_cm` определяет дистанцию, при которой движение считается
        небезопасным и скорость принудительно падает до нуля, а `deceleration_distance_cm`
        задаёт зону, в которой скорость плавно уменьшается по мере приближения к препятствию
        или к целевому расстоянию.

        Параметр `base_speed_percent` задаёт базовую скорость (в процентах от максимальной),
        если для команды не указано отдельное ограничение, а `update_interval_sec` определяет,
        как часто контроллер обновляет измерения датчика, пересчитывает скорость и оценивает
        пройденный путь.
        """
        self.motor_controller: MotorControllerProtocol = motor_controller
        self.ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self.min_obstacle_distance_cm: float = min_obstacle_distance_cm
        self.deceleration_distance_cm: float = deceleration_distance_cm
        self.base_speed_percent: int = base_speed_percent
        self.update_interval_sec: float = update_interval_sec

        self._is_moving: bool = False
        self._target_distance_cm: Optional[float] = None
        self._traveled_distance_cm: float = 0.0
        self._max_speed_percent: Optional[int] = None
        self._movement_thread: Optional[Thread] = None
        self._stop_event: Event = Event()

    def forward_cm(self, distance_cm: float, max_speed_percent: Optional[int] = None) -> None:
        """Движение машинки вперёд на заданное расстояние с ограничением скорости.

        Метод не блокирует вызывающий поток: фактическое движение выполняется в отдельном
        daemon-потоке. При этом: если предыдущее движение ещё не завершилось, оно принудительно
        останавливается через метод `stop`; внутреннее состояние контроллера сбрасывается и
        подготавливается к новому запуску; целевое расстояние и максимально допустимая скорость
        сохраняются во внутреннем состоянии.
        """
        if self._is_moving:
            self.stop()

        self._target_distance_cm: float = distance_cm
        self._traveled_distance_cm: float = 0.0
        self._max_speed_percent: int = max_speed_percent or self.base_speed_percent
        self._is_moving: bool = True
        self._stop_event.clear()

        self._movement_thread: Thread = Thread(target=self._autonomous_movement, daemon=True)
        self._movement_thread.start()

    def stop(self) -> None:
        """Немедленная остановка машинки.

        Сигнализирует служебному потоку автономного движения о необходимости завершиться и
        вызывает метод `motor_controller.stop` для немедленной остановки моторов. При
        необходимости дожидается завершения потока с тайм-аутом.
        """
        self._is_moving: bool = False
        self._stop_event.set()
        self.motor_controller.stop()

        if self._movement_thread and self._movement_thread.is_alive():
            self._movement_thread.join(timeout=1.0)

        self._movement_thread: Optional[Thread] = None

    def _calculate_speed(self, obstacle_distance_cm: float, remaining_distance_cm: float) -> float:
        """Вычисление скорости с учётом препятствий и оставшейся дистанции до цели.

        Сначала скорость ограничивается по расстоянию до ближайшего препятствия: если объект
        расположен ближе или на пороге `min_obstacle_distance_cm`, функция возвращает 0, в
        диапазоне от `min_obstacle_distance_cm` до `deceleration_distance_cm` скорость линейно
        уменьшается от `_max_speed_percent` до нуля, а при большем расстоянии принимается равной
        `_max_speed_percent`.

        Затем, если оставшаяся дистанция до цели `remaining_distance_cm`, внутри зоны торможения
        по цели дополнительно снижается скорость пропорционально оставшемуся пути (но не ниже
        10% от текущего значения), что позволяет плавно остановиться у заданной точки.
        """
        if obstacle_distance_cm <= self.min_obstacle_distance_cm:
            return 0.0

        speed: float = self._max_speed_percent

        if obstacle_distance_cm <= self.deceleration_distance_cm:
            decel_range: float = self.deceleration_distance_cm - self.min_obstacle_distance_cm
            distance_above_min: float = obstacle_distance_cm - self.min_obstacle_distance_cm
            speed *= max(0.0, min(1.0, distance_above_min / decel_range))

        if remaining_distance_cm and remaining_distance_cm < self.deceleration_distance_cm:
            speed *= max(0.1, min(1.0, remaining_distance_cm / self.deceleration_distance_cm))

        return max(0.0, speed)

    def _estimate_traveled_distance(self, speed_percent: float, time_interval: float) -> float:
        """Оценка пройденного расстояния за заданный интервал времени.

        Использует упрощённую кинематическую модель, предполагающую, что при скорости 100%
        машинка проходит примерно `MAX_SPEED_CM_PER_SEC` сантиметров в секунду, и на основе
        текущего процента скорости и длительности интервала времени возвращает примерное
        пройденное расстояние; точность оценки зависит от калибровки константы
        `MAX_SPEED_CM_PER_SEC` и не учитывает проскальзывания, уклоны и другие внешние факторы.
        """
        return self.MAX_SPEED_CM_PER_SEC * (speed_percent / 100.0) * time_interval

    def _autonomous_movement(self) -> None:
        """Цикл автономного движения вперёд с контролем препятствий и целевой дистанции.

        Выполняется в отдельном daemon-потоке: на каждой итерации считывает расстояние до
        препятствия и оставшуюся до цели дистанцию, по этим данным рассчитывает безопасную
        скорость, при нулевой скорости останавливает моторы и делает паузу, при положительной —
        задаёт движение вперёд и обновляет оценку пройденного пути; цикл продолжается, пока
        движение активно и не запрошена остановка, при любой ошибке или завершении работы в
        блоке `finally` гарантированно останавливает моторы и помечает контроллер как
        недвижущийся.
        """
        try:
            while self._is_moving and not self._stop_event.is_set():
                obstacle_distance_cm: float = self.ultrasonic_sensor.measure_distance_cm()

                remaining_distance: float = self._target_distance_cm - self._traveled_distance_cm
                if remaining_distance <= 0:
                    break

                current_speed: float = self._calculate_speed(
                    obstacle_distance_cm=obstacle_distance_cm,
                    remaining_distance_cm=remaining_distance,
                )

                if current_speed <= 0.0:
                    self.motor_controller.stop()
                    time.sleep(self.update_interval_sec)
                    continue

                self.motor_controller.move_forward(speed_percent=int(current_speed))
                self._traveled_distance_cm += self._estimate_traveled_distance(
                    speed_percent=current_speed,
                    time_interval=self.update_interval_sec,
                )

                time.sleep(self.update_interval_sec)

        except Exception as exc:
            logger.exception("Ошибка в автономном движении: %s", exc)

        finally:
            self.motor_controller.stop()
            self._is_moving: bool = False
