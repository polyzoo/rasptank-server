from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar


@dataclass(frozen=True, slots=True)
class TrackCommand:
    """Команды для левого и правого борта."""

    left_percent: float
    right_percent: float


@dataclass(frozen=True, slots=True)
class ChassisVelocity:
    """Скорость корпуса машинки."""

    linear_speed_cm_per_sec: float
    angular_speed_deg_per_sec: float


@dataclass(frozen=True, slots=True)
class DifferentialDriveKinematics:
    """Кинематическая связь корпуса машинки и её бортов."""

    track_width_cm: float
    left_track_max_speed_cm_per_sec: float
    right_track_max_speed_cm_per_sec: float
    max_command_percent: float = 100.0

    # Половина базы между бортами из формулы v_R = v + omega * l / 2 и v_L = v - omega * l / 2.
    HALF_TRACK_WIDTH_DIVISOR: ClassVar[float] = 2.0

    def to_track_command(
        self,
        linear_speed_cm_per_sec: float,
        angular_speed_deg_per_sec: float,
    ) -> TrackCommand:
        """Перевести желаемую скорость корпуса в команды левого и правого борта."""
        angular_speed_rad_per_sec: float = math.radians(angular_speed_deg_per_sec)
        half_track_width_cm: float = self.track_width_cm / self.HALF_TRACK_WIDTH_DIVISOR

        right_track_speed_cm_per_sec: float = (
            linear_speed_cm_per_sec + angular_speed_rad_per_sec * half_track_width_cm
        )

        left_track_speed_cm_per_sec: float = (
            linear_speed_cm_per_sec - angular_speed_rad_per_sec * half_track_width_cm
        )

        left_percent: float = self._speed_to_percent(
            left_track_speed_cm_per_sec,
            self.left_track_max_speed_cm_per_sec,
        )

        right_percent: float = self._speed_to_percent(
            right_track_speed_cm_per_sec,
            self.right_track_max_speed_cm_per_sec,
        )

        return self._normalize_track_command(
            TrackCommand(left_percent=left_percent, right_percent=right_percent)
        )

    def to_chassis_velocity(
        self,
        left_percent: float,
        right_percent: float,
    ) -> ChassisVelocity:
        """Перевести команды бортов в линейную и угловую скорость корпуса."""
        left_track_speed_cm_per_sec: float = self._percent_to_speed(
            left_percent,
            self.left_track_max_speed_cm_per_sec,
        )

        right_track_speed_cm_per_sec: float = self._percent_to_speed(
            right_percent,
            self.right_track_max_speed_cm_per_sec,
        )

        linear_speed_cm_per_sec: float = (
            right_track_speed_cm_per_sec + left_track_speed_cm_per_sec
        ) / self.HALF_TRACK_WIDTH_DIVISOR

        angular_speed_rad_per_sec: float = (
            right_track_speed_cm_per_sec - left_track_speed_cm_per_sec
        ) / self.track_width_cm

        return ChassisVelocity(
            linear_speed_cm_per_sec=linear_speed_cm_per_sec,
            angular_speed_deg_per_sec=math.degrees(angular_speed_rad_per_sec),
        )

    def _normalize_track_command(self, command: TrackCommand) -> TrackCommand:
        """Ограничить команды допустимым диапазоном с сохранением соотношения."""
        max_abs_value: float = max(
            abs(command.left_percent),
            abs(command.right_percent),
            self.max_command_percent,
        )
        if max_abs_value <= self.max_command_percent:
            return command

        scale: float = self.max_command_percent / max_abs_value
        return TrackCommand(
            left_percent=command.left_percent * scale,
            right_percent=command.right_percent * scale,
        )

    def _speed_to_percent(self, speed_cm_per_sec: float, max_speed_cm_per_sec: float) -> float:
        """Перевести скорость борта в команду в процентах."""
        return (speed_cm_per_sec / max_speed_cm_per_sec) * self.max_command_percent

    def _percent_to_speed(self, percent: float, max_speed_cm_per_sec: float) -> float:
        """Перевести команду в процентах в скорость борта."""
        return (percent / self.max_command_percent) * max_speed_cm_per_sec
