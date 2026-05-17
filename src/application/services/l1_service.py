from __future__ import annotations

import logging
import threading
import time
from typing import ClassVar

from src.application.protocols import (
    GyroscopeProtocol,
    HeadServoProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)
from src.application.services.l1_models import L1SensorState, L1TrackCommand

logger: logging.Logger = logging.getLogger(__name__)


class L1Service:
    """Уровень L1 для прямого доступа к устройствам."""

    # Порог предупреждения о медленном полном опросе датчиков.
    SLOW_SENSOR_READ_TOTAL_MS: ClassVar[float] = 200.0
    # Порог предупреждения о медленном измерении ультразвукового датчика.
    SLOW_ULTRASONIC_RELATIVE_MS: ClassVar[float] = 120.0

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        gyroscope: GyroscopeProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        head_servo: HeadServoProtocol | None = None,
    ) -> None:
        """Сохранить устройства L1."""
        self._motor_controller: MotorControllerProtocol = motor_controller
        self._gyroscope: GyroscopeProtocol = gyroscope
        self._ultrasonic_sensor: UltrasonicSensorProtocol = ultrasonic_sensor
        self._head_servo: HeadServoProtocol | None = head_servo

    def start_imu(self, *, calibrate: bool = True) -> None:
        """Запустить IMU."""
        self._gyroscope.start(calibrate=calibrate)

    def stop_imu(self) -> None:
        """Остановить IMU."""
        self._gyroscope.stop()

    def apply_track_command(self, command: L1TrackCommand) -> None:
        """Передать команду бортов на моторы."""
        self._motor_controller.set_tracks(
            left_speed_percent=command.left_percent,
            right_speed_percent=command.right_percent,
        )

    def stop_motion(self) -> None:
        """Остановить оба борта."""
        self._motor_controller.stop()

    def read_sensors(self) -> L1SensorState:
        """Прочитать доступные данные датчиков без вычисления положения."""
        t_start: float = time.perf_counter()
        accel_x_m_s2, accel_y_m_s2, accel_z_m_s2 = self._gyroscope.get_acceleration_xyz_m_s2()
        t_after_accel: float = time.perf_counter()
        angular_speed_z_deg_per_sec: float = self._gyroscope.get_angular_speed_z_deg_per_sec()
        t_after_gyro: float = time.perf_counter()
        distance_cm: float = self._ultrasonic_sensor.measure_distance_cm()
        t_end: float = time.perf_counter()

        imu_ms: float = (t_after_gyro - t_start) * 1000.0
        ultrasonic_ms: float = (t_end - t_after_gyro) * 1000.0
        total_ms: float = (t_end - t_start) * 1000.0

        logger.debug(
            "L1.read_sensors thread=%s accel_ms=%.2f gyro_ms=%.2f imu_total_ms=%.2f "
            "ultrasonic_ms=%.2f total_ms=%.2f dist_cm=%.1f",
            threading.current_thread().name,
            (t_after_accel - t_start) * 1000.0,
            (t_after_gyro - t_after_accel) * 1000.0,
            imu_ms,
            ultrasonic_ms,
            total_ms,
            distance_cm,
        )

        slow_total: bool = total_ms >= self.SLOW_SENSOR_READ_TOTAL_MS
        slow_us: bool = ultrasonic_ms >= self.SLOW_ULTRASONIC_RELATIVE_MS
        if slow_total or slow_us:
            logger.warning(
                "L1.read_sensors медленно thread=%s total_ms=%.1f imu_ms=%.1f ultrasonic_ms=%.1f "
                "(узкое место часто ultrasonic/ECHO или конкуренция за GPIO)",
                threading.current_thread().name,
                total_ms,
                imu_ms,
                ultrasonic_ms,
            )

        return L1SensorState(
            angular_speed_z_deg_per_sec=angular_speed_z_deg_per_sec,
            accel_x_m_s2=accel_x_m_s2,
            accel_y_m_s2=accel_y_m_s2,
            accel_z_m_s2=accel_z_m_s2,
            distance_cm=distance_cm,
        )

    def set_head_angle(self, angle_deg: float) -> None:
        """Передать угол на сервопривод головы."""
        if self._head_servo is None:
            return
        self._head_servo.set_angle(angle_deg)

    def destroy(self, *, release_devices: bool = True) -> None:
        """Освободить ресурсы устройств L1."""
        if not release_devices:
            return
        self._motor_controller.destroy()
        self._gyroscope.destroy()
        self._ultrasonic_sensor.destroy()
        if self._head_servo is not None:
            self._head_servo.destroy()
