from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MotionConfig:
    """Настройки движения для фасада контроллера."""

    min_obstacle_distance_cm: float
    deceleration_distance_cm: float
    base_speed_percent: int
    turn_speed_percent: int
    turn_slowdown_remaining_deg: float
    turn_creep_speed_percent: int
    turn_angle_trim_deg: float
    last_turn_angle_trim_deg: float
    max_speed_cm_per_sec: float
    update_interval_sec: float
    heading_hold_enabled: bool
    heading_hold_kp: float
    heading_hold_steer_max: int
    heading_hold_deadband_deg: float
    heading_hold_steer_speed_ratio: float
    heading_hold_min_speed_percent: float
    heading_hold_steer_cap_min_spd_percent: float
    heading_hold_steer_trim: int
    heading_hold_invert_steer: bool
    forward_soft_start_sec: float
    turn_check_interval_sec: float
    turn_obstacle_check_interval_sec: float
    turn_timeout_per_deg: float
    turn_timeout_min: float
