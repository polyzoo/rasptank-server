from __future__ import annotations

import math

import pytest

from src.infrastructures.ekf_imu import EkfImu


def test_ekf_stationary_zupt_reduces_yaw_drift() -> None:
    """При нулевой угловой скорости и стабильном g yaw почти не уезжает."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    dt = 0.01
    g = 9.80665
    for _ in range(80):
        ekf.predict(0.0, 0.0, 0.0, dt)
        ekf.update(0.0, 0.0, g)
    assert ekf.stationary is True
    assert abs(ekf.yaw) < 0.05


def test_ekf_predict_updates_yaw_when_rotating() -> None:
    """При вращении только вокруг Z yaw меняется ожидаемо."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    dt = 0.1
    wz = math.radians(90.0)
    ekf.predict(0.0, 0.0, wz, dt)
    ekf.update(0.0, 0.0, 9.80665)
    assert ekf.yaw > math.radians(8.0)


def test_ekf_reset_clears_state() -> None:
    """reset обнуляет yaw, bias и флаг стационарности."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    ekf.predict(0.0, 0.0, math.radians(10.0), 0.05)
    ekf.reset()
    assert ekf.yaw == 0.0
    assert ekf.stationary is False
    assert ekf.gyro_bias[2] == 0.0


def test_ekf_predict_zero_dt_is_noop() -> None:
    """При dt <= 0 predict не меняет состояние."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.5, (0.0, 0.0, 0.0))
    yaw_before = ekf.yaw
    ekf.predict(0.0, 0.0, math.radians(100.0), 0.0)
    assert ekf.yaw == yaw_before


def test_ekf_exits_stationary_on_large_gz() -> None:
    """В ZUPT выходим, если |ω_z| превышает zupt_exit."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    for _ in range(6):
        ekf.predict(0.0, 0.0, 0.0, 0.01)
        ekf.update(0.0, 0.0, 9.80665)
    assert ekf.stationary is True
    ekf.predict(0.0, 0.0, 0.05, 0.01)
    assert ekf.stationary is False


def test_ekf_update_large_accel_exits_zupt() -> None:
    """Сильное ускорение срывает режим стационарности."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    for _ in range(6):
        ekf.predict(0.0, 0.0, 0.0, 0.01)
        ekf.update(0.0, 0.0, 9.80665)
    assert ekf.stationary is True
    ekf.update(40.0, 0.0, 0.0)
    assert ekf.stationary is False


def test_ekf_update_skips_orientation_when_accel_gate() -> None:
    """При ‖a‖ далеко от g обновление ориентации по акселю пропускается."""
    ekf = EkfImu(accel_gate=0.3)
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    ekf.update(0.0, 0.0, 0.0)
    assert ekf.roll == 0.0
    assert ekf.pitch == 0.0


def test_ekf_update_increments_zupt_count_when_almost_flatline() -> None:
    """Пока не стационарны, малый |‖a‖−g| наращивает счётчик подтверждения."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    g = 9.80665
    ekf.update(0.0, 0.0, g)
    assert ekf._zupt_count == 1  # noqa: SLF001


def test_ekf_predict_clamps_tiny_covariance() -> None:
    """В ветке ZUPT ковариации не опускаются ниже пола."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    ekf._stationary = True  # noqa: SLF001
    ekf._P_yaw = 1e-11  # noqa: SLF001
    ekf._P_bias = 1e-11  # noqa: SLF001
    ekf._P_cross = 0.0  # noqa: SLF001
    ekf.predict(0.0, 0.0, 0.0, 0.01)
    assert ekf._P_yaw >= 1e-10  # noqa: SLF001
    assert ekf._P_bias >= 1e-10  # noqa: SLF001


def test_ekf_accessors_after_valid_update() -> None:
    """roll/pitch, gyro_bias и get_euler_deg отражают update при норме ≈ g."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    ekf.update(0.0, 0.0, 9.80665)
    _ = ekf.roll
    _ = ekf.pitch
    assert ekf.gyro_bias[0] == 0.0
    r_d, p_d, y_d = ekf.get_euler_deg()
    assert isinstance(r_d, float)
    assert isinstance(p_d, float)
    assert isinstance(y_d, float)


@pytest.mark.parametrize("raw_gz", [10.0, -10.0])
def test_ekf_else_branch_resets_zupt_count_on_motion(raw_gz: float) -> None:
    """При |ω_z| выше zupt_enter счётчик подтверждения стационарности сбрасывается."""
    ekf = EkfImu()
    ekf.init_from_calibration(0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
    ekf._zupt_count = 3  # noqa: SLF001
    ekf.predict(0.0, 0.0, math.radians(raw_gz), 0.01)
    assert ekf._zupt_count == 0  # noqa: SLF001
