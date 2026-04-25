from __future__ import annotations

from src.application.protocols import (
    GyroscopeProtocol,
    HeadServoProtocol,
    MotorControllerProtocol,
    UltrasonicSensorProtocol,
)
from src.application.services.l1_models import L1SensorState, L1TrackCommand


class L1Service:
    """Чистый нижний уровень нового контура без вычисления движения."""

    def __init__(
        self,
        motor_controller: MotorControllerProtocol,
        gyroscope: GyroscopeProtocol,
        ultrasonic_sensor: UltrasonicSensorProtocol,
        head_servo: HeadServoProtocol | None = None,
    ) -> None:
        """Сохранить устройства нижнего уровня."""
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
        """Сразу передать команду левого и правого борта на моторы."""
        self._motor_controller.set_tracks(
            left_speed_percent=command.left_percent,
            right_speed_percent=command.right_percent,
        )

    def stop_motion(self) -> None:
        """Остановить оба борта."""
        self._motor_controller.stop()

    def read_sensors(self) -> L1SensorState:
        """Прочитать доступные данные датчиков без вычисления положения."""
        accel_x_m_s2, accel_y_m_s2, accel_z_m_s2 = self._gyroscope.get_acceleration_xyz_m_s2()
        return L1SensorState(
            angular_speed_z_deg_per_sec=self._gyroscope.get_angular_speed_z_deg_per_sec(),
            accel_x_m_s2=accel_x_m_s2,
            accel_y_m_s2=accel_y_m_s2,
            accel_z_m_s2=accel_z_m_s2,
            distance_cm=self._ultrasonic_sensor.measure_distance_cm(),
        )

    def set_head_angle(self, angle_deg: float) -> None:
        """Передать угол на сервопривод головы."""
        if self._head_servo is None:
            return
        self._head_servo.set_angle(angle_deg)

    def destroy(self, *, release_devices: bool = True) -> None:
        """Освободить ресурсы устройств нижнего уровня."""
        if not release_devices:
            return
        self._motor_controller.destroy()
        self._gyroscope.destroy()
        self._ultrasonic_sensor.destroy()
        if self._head_servo is not None:
            self._head_servo.destroy()
