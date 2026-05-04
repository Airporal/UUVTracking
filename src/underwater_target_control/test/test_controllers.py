"""Unit tests for underwater_target_control — controllers."""

from __future__ import annotations

import pytest

from underwater_target_control.controller_base import (
    ControlCommand,
    ControllerBase,
    TargetObservation,
)
from underwater_target_control.ibvs_controller import IBVSController
from underwater_target_control.pid_controller import PIDController, _PID


# ---------------------------------------------------------------------------
# Concrete stub for testing ControllerBase helpers
# ---------------------------------------------------------------------------


class _StubController(ControllerBase):
    def compute_command(self, observation, dt):
        return ControlCommand()

    def reset(self):
        pass


# ---------------------------------------------------------------------------
# ControllerBase tests
# ---------------------------------------------------------------------------


class TestControllerBase:
    def test_saturate_within_limits(self):
        ctrl = _StubController(max_linear_vel=0.5, max_angular_vel=0.5)
        cmd = ControlCommand(vx=0.3, vy=-0.2, vz=0.1, wx=0.0, wy=0.0, wz=0.4)
        sat = ctrl._saturate(cmd)
        assert sat.vx == pytest.approx(0.3)
        assert sat.vy == pytest.approx(-0.2)
        assert sat.wz == pytest.approx(0.4)

    def test_saturate_exceeds_limits(self):
        ctrl = _StubController(max_linear_vel=0.5, max_angular_vel=0.5)
        cmd = ControlCommand(vx=2.0, vy=-3.0, vz=0.0, wx=0.0, wy=0.0, wz=1.5)
        sat = ctrl._saturate(cmd)
        assert sat.vx == pytest.approx(0.5)
        assert sat.vy == pytest.approx(-0.5)
        assert sat.wz == pytest.approx(0.5)

    def test_max_linear_vel_setter_valid(self):
        ctrl = _StubController()
        ctrl.max_linear_vel = 1.0
        assert ctrl.max_linear_vel == pytest.approx(1.0)

    def test_max_linear_vel_setter_invalid(self):
        ctrl = _StubController()
        with pytest.raises(ValueError):
            ctrl.max_linear_vel = -0.1


# ---------------------------------------------------------------------------
# IBVSController tests
# ---------------------------------------------------------------------------


class TestIBVSController:
    def test_zero_command_when_not_valid(self):
        ctrl = IBVSController()
        obs = TargetObservation(is_valid=False)
        cmd = ctrl.compute_command(obs, dt=0.05)
        assert cmd.vx == pytest.approx(0.0)
        assert cmd.wz == pytest.approx(0.0)

    def test_centred_target_zero_command(self):
        ctrl = IBVSController(lambda_gain=0.5, deadband=0.05)
        obs = TargetObservation(image_x=0.0, image_y=0.0, is_valid=True)
        cmd = ctrl.compute_command(obs, dt=0.05)
        assert cmd.wz == pytest.approx(0.0)
        assert cmd.vy == pytest.approx(0.0)

    def test_right_target_produces_positive_yaw(self):
        ctrl = IBVSController(lambda_gain=0.5, deadband=0.0)
        # Target is to the right (image_x > 0); we expect yaw toward right (wz < 0)
        obs = TargetObservation(image_x=0.5, image_y=0.0, is_valid=True)
        cmd = ctrl.compute_command(obs, dt=0.05)
        assert cmd.wz < 0.0

    def test_command_saturated(self):
        ctrl = IBVSController(
            lambda_gain=10.0, max_angular_vel=0.5, deadband=0.0
        )
        obs = TargetObservation(image_x=1.0, image_y=0.0, is_valid=True)
        cmd = ctrl.compute_command(obs, dt=0.05)
        assert abs(cmd.wz) <= 0.5

    def test_reset_is_no_op(self):
        ctrl = IBVSController()
        ctrl.reset()  # should not raise


# ---------------------------------------------------------------------------
# PIDController / _PID tests
# ---------------------------------------------------------------------------


class TestPID:
    def test_zero_error_zero_output(self):
        pid = _PID(kp=1.0, ki=0.0, kd=0.0, output_limit=10.0)
        assert pid.step(0.0, 0.1) == pytest.approx(0.0)

    def test_proportional_output(self):
        pid = _PID(kp=2.0, ki=0.0, kd=0.0, output_limit=10.0)
        out = pid.step(1.0, 0.1)
        assert out == pytest.approx(2.0)

    def test_output_clamped(self):
        pid = _PID(kp=100.0, ki=0.0, kd=0.0, output_limit=1.0)
        out = pid.step(1.0, 0.1)
        assert out == pytest.approx(1.0)

    def test_integral_accumulates(self):
        pid = _PID(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0)
        pid.step(1.0, 0.1)  # integral = 0.1
        out = pid.step(1.0, 0.1)  # integral = 0.2
        assert out == pytest.approx(0.2, abs=1e-6)

    def test_reset_clears_state(self):
        pid = _PID(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0)
        pid.step(1.0, 1.0)  # integral = 1.0
        pid.reset()
        out = pid.step(1.0, 1.0)  # after reset, integral starts fresh = 1.0
        assert out == pytest.approx(1.0, abs=1e-6)


class TestPIDController:
    def test_zero_command_when_not_valid(self):
        ctrl = PIDController()
        obs = TargetObservation(is_valid=False)
        cmd = ctrl.compute_command(obs, dt=0.05)
        assert cmd.vx == pytest.approx(0.0)
        assert cmd.wz == pytest.approx(0.0)

    def test_centred_target_zero_command(self):
        ctrl = PIDController(yaw_kp=0.8, heave_kp=0.5, deadband=0.05)
        obs = TargetObservation(image_x=0.0, image_y=0.0, is_valid=True)
        cmd = ctrl.compute_command(obs, dt=0.05)
        assert cmd.wz == pytest.approx(0.0)
        assert cmd.vz == pytest.approx(0.0)

    def test_right_target_produces_negative_wz(self):
        ctrl = PIDController(yaw_kp=0.8, deadband=0.0)
        obs = TargetObservation(image_x=0.5, image_y=0.0, is_valid=True)
        cmd = ctrl.compute_command(obs, dt=0.05)
        assert cmd.wz < 0.0

    def test_command_saturated(self):
        ctrl = PIDController(yaw_kp=100.0, max_angular_vel=0.5, deadband=0.0)
        obs = TargetObservation(image_x=1.0, image_y=0.0, is_valid=True)
        cmd = ctrl.compute_command(obs, dt=0.05)
        assert abs(cmd.wz) <= 0.5

    def test_reset(self):
        ctrl = PIDController()
        obs = TargetObservation(image_x=0.5, is_valid=True)
        ctrl.compute_command(obs, dt=0.1)
        ctrl.reset()  # should not raise
