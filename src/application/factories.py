from __future__ import annotations

from dataclasses import dataclass

from src.application.protocols import MotorControllerProtocol
from src.application.services.drive_controller import DriveController
from src.application.services.goal_point_controller import GoalPointController
from src.application.services.isolated_motion_service import IsolatedMotionService
from src.application.services.kinematics import DifferentialDriveKinematics
from src.application.services.l1_service import L1Service
from src.application.services.l2_service import L2Service
from src.application.services.l3_service import L3Service
from src.application.services.motion_config import MotionConfig
from src.application.services.motion_events import MotionEventHub
from src.application.services.path_planner import PathPlanner
from src.application.services.pose_estimator import PoseEstimator
from src.application.services.velocity_command_controller import VelocityCommandController
from src.config.settings import Settings
from src.infrastructures.head_servo import HeadServoController
from src.infrastructures.imu import IMUSensor
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import UltrasonicSensor


@dataclass(slots=True)
class SharedMotionHardware:
    """Один набор GPIO/I²C-устройств на процесс (моторы, IMU, УЗ-датчик, серво головы)."""

    motor_controller: MotorController
    gyroscope: IMUSensor
    ultrasonic_sensor: UltrasonicSensor
    head_servo: HeadServoController

    def destroy(self) -> None:
        """Освободить железо в том же порядке, что и DriveController.destroy."""
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()
        self.gyroscope.destroy()
        self.head_servo.destroy()


def create_shared_motion_hardware(settings: Settings) -> SharedMotionHardware:
    """Создать единственный экземпляр нижнеуровневого железа для приложения."""
    return SharedMotionHardware(
        motor_controller=MotorController(
            tl_left_offset=settings.tl_left_offset,
            tl_right_offset=settings.tl_right_offset,
            invert_line_motion=settings.invert_line_motion,
        ),
        gyroscope=IMUSensor(),
        ultrasonic_sensor=UltrasonicSensor(),
        head_servo=HeadServoController(
            channel=settings.head_servo_channel,
            home_angle_deg=settings.head_servo_home_angle_deg,
        ),
    )


def create_drive_controller(
    settings: Settings,
    motion_events: MotionEventHub | None,
    hardware: SharedMotionHardware,
) -> DriveController:
    """Собирает контроллер движения поверх общего железа."""
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
        motor_controller=hardware.motor_controller,
        gyroscope=hardware.gyroscope,
        ultrasonic_sensor=hardware.ultrasonic_sensor,
        config=config,
        motion_events=motion_events,
        head_servo=hardware.head_servo,
        head_servo_home_angle_deg=settings.head_servo_home_angle_deg,
        release_gyroscope_after_route=False,
    )


def create_differential_drive_kinematics(settings: Settings) -> DifferentialDriveKinematics:
    """Собрать кинематику корпуса и бортов по настройкам приложения."""
    return DifferentialDriveKinematics(
        track_width_cm=settings.track_width_cm,
        left_track_max_speed_cm_per_sec=settings.left_track_max_speed_cm_per_sec,
        right_track_max_speed_cm_per_sec=settings.right_track_max_speed_cm_per_sec,
    )


def create_pose_estimator() -> PoseEstimator:
    """Создать оценщик положения и скорости машинки."""
    return PoseEstimator()


def create_velocity_command_controller(
    settings: Settings,
    motor_controller: MotorControllerProtocol,
) -> VelocityCommandController:
    """Собрать контроллер команд скорости корпуса поверх нижнего уровня."""
    return VelocityCommandController(
        motor_controller=motor_controller,
        kinematics=create_differential_drive_kinematics(settings),
    )


def create_l2_service(
    settings: Settings,
    motor_controller: MotorControllerProtocol,
) -> L2Service:
    """Собрать изолированный математический слой нового контура."""
    kinematics: DifferentialDriveKinematics = create_differential_drive_kinematics(settings)
    pose_estimator: PoseEstimator = create_pose_estimator()
    velocity_controller: VelocityCommandController = VelocityCommandController(
        motor_controller=motor_controller,
        kinematics=kinematics,
    )
    return L2Service(
        kinematics=kinematics,
        pose_estimator=pose_estimator,
        velocity_controller=velocity_controller,
    )


def create_l1_service(settings: Settings) -> L1Service:
    """Собрать чистый нижний уровень нового контура."""
    imu_sensor: IMUSensor = IMUSensor()
    ultrasonic_sensor: UltrasonicSensor = UltrasonicSensor()
    head_servo: HeadServoController = HeadServoController(
        channel=settings.head_servo_channel,
        home_angle_deg=settings.head_servo_home_angle_deg,
    )
    motor_controller: MotorController = MotorController(
        tl_left_offset=settings.tl_left_offset,
        tl_right_offset=settings.tl_right_offset,
        invert_line_motion=settings.invert_line_motion,
    )
    return L1Service(
        motor_controller=motor_controller,
        gyroscope=imu_sensor,
        ultrasonic_sensor=ultrasonic_sensor,
        head_servo=head_servo,
    )


def create_goal_point_controller(settings: Settings) -> GoalPointController:
    """Собрать контроллер движения к целевой точке."""
    return GoalPointController(
        position_tolerance_cm=settings.l3_position_tolerance_cm,
        linear_speed_gain=settings.l3_linear_speed_gain,
        angular_speed_gain=settings.l3_angular_speed_gain,
        max_linear_speed_cm_per_sec=settings.l3_max_linear_speed_cm_per_sec,
        max_angular_speed_deg_per_sec=settings.l3_max_angular_speed_deg_per_sec,
        obstacle_stop_distance_cm=settings.l3_obstacle_stop_distance_cm,
        obstacle_slowdown_distance_cm=settings.l3_obstacle_slowdown_distance_cm,
    )


def create_path_planner(settings: Settings) -> PathPlanner:
    """Собрать планировщик обходного маршрута для L3."""
    return PathPlanner(
        obstacle_clearance_cm=settings.l3_planner_obstacle_clearance_cm,
        max_detour_offset_cm=settings.l3_planner_max_detour_offset_cm,
        max_waypoints=settings.l3_planner_max_waypoints,
    )


def create_l3_service(settings: Settings, l2_service: L2Service) -> L3Service:
    """Собрать изолированный верхний уровень нового контура."""
    return L3Service(
        goal_point_controller=create_goal_point_controller(settings),
        path_planner=create_path_planner(settings),
        l2_service=l2_service,
        unknown_obstacle_radius_cm=settings.l3_unknown_obstacle_radius_cm,
    )


def create_isolated_motion_service(
    settings: Settings,
    hardware: SharedMotionHardware,
) -> IsolatedMotionService:
    """Собрать общий координирующий сервис контура L1-L3 на том же железе, что и DriveController."""
    l1_service: L1Service = L1Service(
        motor_controller=hardware.motor_controller,
        gyroscope=hardware.gyroscope,
        ultrasonic_sensor=hardware.ultrasonic_sensor,
        head_servo=hardware.head_servo,
    )
    l2_service: L2Service = create_l2_service(settings, hardware.motor_controller)
    l3_service: L3Service = create_l3_service(settings, l2_service)

    return IsolatedMotionService(
        l1_service=l1_service,
        l2_service=l2_service,
        l3_service=l3_service,
        update_interval_sec=settings.update_interval_sec,
    )
