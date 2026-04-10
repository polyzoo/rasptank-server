from __future__ import annotations

from src.application.services.drive_controller import DriveController
from src.application.services.motion_config import MotionConfig
from src.application.services.motion_events import MotionEventHub
from src.config.settings import Settings
from src.infrastructures.imu import IMUSensor
from src.infrastructures.head_servo import HeadServoController
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor


def create_drive_controller(
    settings: Settings,
    motion_events: MotionEventHub | None = None,
) -> DriveController:
    """Собирает контроллер движения со всеми инфраструктурными зависимостями."""
    imu_sensor: IMUSensor = IMUSensor()
    ultrasonic_sensor: UltrasonicSensor = UltrasonicSensor()
    head_servo: HeadServoController = HeadServoController(
        channel=settings.head_servo_channel,
        home_angle_deg=settings.head_servo_home_angle_deg,
    )
    motor_controller: MotorController = MotorController(
        tl_left_offset=settings.tl_left_offset,
        tl_right_offset=settings.tl_right_offset,
    )

    config: MotionConfig = MotionConfig(
        min_obstacle_distance_cm=settings.min_obstacle_distance_cm,
        deceleration_distance_cm=settings.deceleration_distance_cm,
        base_speed_percent=settings.base_speed_percent,
        turn_speed_percent=settings.turn_speed_percent,
        turn_slowdown_remaining_deg=settings.turn_slowdown_remaining_deg,
        turn_creep_speed_percent=settings.turn_creep_speed_percent,
        turn_angle_trim_deg=settings.turn_angle_trim_deg,
        last_turn_angle_trim_deg=settings.last_turn_angle_trim_deg,
        max_speed_cm_per_sec=settings.max_speed_cm_per_sec,
        update_interval_sec=settings.update_interval_sec,
        avoidance_scan_angle_deg=settings.avoidance_scan_angle_deg,
        avoidance_side_step_cm=settings.avoidance_side_step_cm,
        avoidance_forward_step_cm=settings.avoidance_forward_step_cm,
        avoidance_rejoin_step_cm=settings.avoidance_rejoin_step_cm,
        avoidance_max_attempts=settings.avoidance_max_attempts,
        avoidance_confirm_readings=settings.avoidance_confirm_readings,
        avoidance_min_side_clearance_cm=settings.avoidance_min_side_clearance_cm,
        avoidance_max_lateral_offset_cm=settings.avoidance_max_lateral_offset_cm,
        avoidance_max_bypass_distance_cm=settings.avoidance_max_bypass_distance_cm,
        heading_hold_enabled=settings.heading_hold_enabled,
        heading_hold_kp=settings.heading_hold_kp,
        heading_hold_steer_max=settings.heading_hold_steer_max,
        heading_hold_deadband_deg=settings.heading_hold_deadband_deg,
        heading_hold_steer_speed_ratio=settings.heading_hold_steer_speed_ratio,
        heading_hold_min_speed_percent=settings.heading_hold_min_speed_percent,
        heading_hold_steer_cap_min_spd_percent=settings.heading_hold_steer_cap_min_speed_percent,
        heading_hold_steer_trim=settings.heading_hold_steer_trim,
        heading_hold_invert_steer=settings.heading_hold_invert_steer,
        forward_soft_start_sec=settings.forward_soft_start_sec,
        turn_check_interval_sec=settings.turn_check_interval_sec,
        turn_obstacle_check_interval_sec=settings.turn_obstacle_check_interval_sec,
        turn_timeout_per_deg=settings.turn_timeout_per_deg,
        turn_timeout_min=settings.turn_timeout_min,
    )

    return DriveController(
        motor_controller=motor_controller,
        gyroscope=imu_sensor,
        ultrasonic_sensor=ultrasonic_sensor,
        config=config,
        motion_events=motion_events,
        head_servo=head_servo,
        head_servo_home_angle_deg=settings.head_servo_home_angle_deg,
    )
