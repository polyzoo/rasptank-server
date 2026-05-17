from __future__ import annotations

from dataclasses import dataclass

from src.application.protocols import MotorControllerProtocol, UltrasonicSensorProtocol
from src.application.services.goal_point_controller import GoalPointController
from src.application.services.isolated_motion_service import IsolatedMotionService
from src.application.services.kinematics import DifferentialDriveKinematics
from src.application.services.l1_service import L1Service
from src.application.services.l2_feedback_controller import L2FeedbackController
from src.application.services.l2_service import L2Service
from src.application.services.l2_state_space_controller import L2StateSpaceController
from src.application.services.l3_service import L3Service
from src.application.services.path_planner import PathPlanner
from src.application.services.pose_estimator import PoseEstimator
from src.application.services.velocity_command_controller import VelocityCommandController
from src.config.settings import Settings
from src.infrastructures.head_servo import HeadServoController
from src.infrastructures.imu import IMUSensor
from src.infrastructures.motor import MotorController
from src.infrastructures.ultrasonic import DisabledUltrasonicSensor, UltrasonicSensor


@dataclass(slots=True)
class SharedMotionHardware:
    """Общий набор устройств движения для контура L1-L3."""

    motor_controller: MotorController
    gyroscope: IMUSensor
    ultrasonic_sensor: UltrasonicSensorProtocol
    head_servo: HeadServoController

    def destroy(self) -> None:
        """Освободить общие устройства приложения."""
        self.motor_controller.destroy()
        self.ultrasonic_sensor.destroy()
        self.gyroscope.destroy()
        self.head_servo.destroy()


def create_shared_motion_hardware(settings: Settings) -> SharedMotionHardware:
    """Создать общий набор устройств движения."""
    ultrasonic_sensor: UltrasonicSensorProtocol = _create_ultrasonic_sensor(settings)
    return SharedMotionHardware(
        motor_controller=MotorController(
            m1_direction=settings.m1_direction,
            m2_direction=settings.m2_direction,
        ),
        gyroscope=IMUSensor(
            gyro_yaw_integration_deadband_deg_per_sec=settings.gyro_yaw_integration_deadband_deg_per_sec,
            mpu6050_dlpf_cfg=settings.imu_mpu6050_dlpf_cfg,
            mpu6050_smplrt_div=settings.imu_mpu6050_smplrt_div,
            accel_ema_alpha=settings.imu_accel_ema_alpha,
            gyro_ema_alpha=settings.imu_gyro_ema_alpha,
            ekf_enabled=settings.imu_ekf_enabled,
            ekf_q_angle=settings.imu_ekf_q_angle,
            ekf_q_bias=settings.imu_ekf_q_bias,
            ekf_r_accel=settings.imu_ekf_r_accel,
            ekf_accel_gate=settings.imu_ekf_accel_gate,
        ),
        ultrasonic_sensor=ultrasonic_sensor,
        head_servo=HeadServoController(
            channel=settings.head_servo_channel,
            home_angle_deg=settings.head_servo_home_angle_deg,
        ),
    )


def create_differential_drive_kinematics(settings: Settings) -> DifferentialDriveKinematics:
    """Создать кинематику корпуса и бортов."""
    return DifferentialDriveKinematics(
        track_width_cm=settings.track_width_cm,
        left_track_max_speed_cm_per_sec=settings.left_track_max_speed_cm_per_sec,
        right_track_max_speed_cm_per_sec=settings.right_track_max_speed_cm_per_sec,
    )


def create_pose_estimator() -> PoseEstimator:
    """Создать оценщик позы корпуса."""
    return PoseEstimator()


def create_velocity_command_controller(
    settings: Settings,
    motor_controller: MotorControllerProtocol,
) -> VelocityCommandController:
    """Создать контроллер команд скорости корпуса."""
    return VelocityCommandController(
        motor_controller=motor_controller,
        kinematics=create_differential_drive_kinematics(settings),
    )


def create_l2_service(
    settings: Settings,
    motor_controller: MotorControllerProtocol,
) -> L2Service:
    """Создать уровень L2."""
    kinematics: DifferentialDriveKinematics = create_differential_drive_kinematics(settings)
    pose_estimator: PoseEstimator = create_pose_estimator()
    velocity_controller: VelocityCommandController = VelocityCommandController(
        motor_controller=motor_controller,
        kinematics=kinematics,
    )

    feedback_controller: L2FeedbackController | None = None
    if settings.l2_feedback_enabled:
        feedback_controller = L2FeedbackController()

    l2_service = L2Service(
        kinematics=kinematics,
        pose_estimator=pose_estimator,
        velocity_controller=velocity_controller,
        feedback_controller=feedback_controller,
    )

    if settings.l2_state_space_enabled:
        state_space_controller = L2StateSpaceController(
            t_v=settings.l2_state_space_t_v,
            t_w=settings.l2_state_space_t_w,
        )
        l2_service.enable_state_space_control(state_space_controller)

    return l2_service


def create_l1_service(settings: Settings) -> L1Service:
    """Создать уровень L1 с отдельными устройствами."""
    imu_sensor: IMUSensor = IMUSensor(
        gyro_yaw_integration_deadband_deg_per_sec=settings.gyro_yaw_integration_deadband_deg_per_sec,
        mpu6050_dlpf_cfg=settings.imu_mpu6050_dlpf_cfg,
        mpu6050_smplrt_div=settings.imu_mpu6050_smplrt_div,
        accel_ema_alpha=settings.imu_accel_ema_alpha,
        gyro_ema_alpha=settings.imu_gyro_ema_alpha,
        ekf_enabled=settings.imu_ekf_enabled,
        ekf_q_angle=settings.imu_ekf_q_angle,
        ekf_q_bias=settings.imu_ekf_q_bias,
        ekf_r_accel=settings.imu_ekf_r_accel,
        ekf_accel_gate=settings.imu_ekf_accel_gate,
    )
    ultrasonic_sensor: UltrasonicSensorProtocol = _create_ultrasonic_sensor(settings)
    head_servo: HeadServoController = HeadServoController(
        channel=settings.head_servo_channel,
        home_angle_deg=settings.head_servo_home_angle_deg,
    )
    motor_controller: MotorController = MotorController(
        m1_direction=settings.m1_direction,
        m2_direction=settings.m2_direction,
    )
    return L1Service(
        motor_controller=motor_controller,
        gyroscope=imu_sensor,
        ultrasonic_sensor=ultrasonic_sensor,
        head_servo=head_servo,
    )


def _create_ultrasonic_sensor(settings: Settings) -> UltrasonicSensorProtocol:
    """Создать ультразвуковой датчик или безопасную заглушку."""
    if not settings.ultrasonic_enabled:
        return DisabledUltrasonicSensor()

    sensor = UltrasonicSensor()
    sensor.TRIGGER_PIN = settings.ultrasonic_trigger_pin
    sensor.ECHO_PIN = settings.ultrasonic_echo_pin
    return sensor


def create_goal_point_controller(settings: Settings) -> GoalPointController:
    """Создать контроллер движения к целевой точке."""
    return GoalPointController()


def create_path_planner(settings: Settings) -> PathPlanner:
    """Создать планировщик маршрута L3."""
    return PathPlanner()


def create_l3_service(
    settings: Settings,
    l2_service: L2Service,
) -> L3Service:
    """Создать уровень L3."""
    return L3Service(
        goal_point_controller=create_goal_point_controller(settings),
        path_planner=create_path_planner(settings),
        l2_service=l2_service,
    )


def create_isolated_motion_service(
    settings: Settings,
    hardware: SharedMotionHardware,
) -> IsolatedMotionService:
    """Создать координатор L1-L3 на общих устройствах."""
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
        debug_trace_enabled=settings.l123_debug_trace_enabled,
        debug_trace_every_n_steps=settings.l123_debug_trace_every_n_steps,
    )
