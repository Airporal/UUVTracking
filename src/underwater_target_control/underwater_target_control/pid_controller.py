"""PID-based target-following controller.
test
Two independent PID loops operate in parallel:
  * Horizontal image error (image_x) → yaw rate (wz)
  * Vertical   image error (image_y) → heave    (vz)

An optional surge PID controls stand-off distance when distance
estimates are available.
"""

from __future__ import annotations

from underwater_target_control.controller_base import (
    ControlCommand,
    ControllerBase,
    TargetObservation,
)


class _PID:
    """Scalar PID controller with anti-windup."""

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limit: float = 1.0,
        integral_limit: float = 10.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._first_step: bool = True

    def step(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        p = self.kp * error

        self._integral += error * dt
        # Anti-windup clamping
        self._integral = max(
            -self.integral_limit, min(self.integral_limit, self._integral)
        )
        i = self.ki * self._integral

        if self._first_step:
            d = 0.0
            self._first_step = False
        else:
            d = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_step = True


class PIDController(ControllerBase):
    """PID-based image-plane target-following controller.

    Parameters
    ----------
    yaw_kp, yaw_ki, yaw_kd:
        Gains for the horizontal (image_x → yaw rate) PID loop.
    heave_kp, heave_ki, heave_kd:
        Gains for the vertical (image_y → heave velocity) PID loop.
    surge_kp, surge_ki, surge_kd:
        Gains for the distance-to-target (surge velocity) PID loop.
    desired_distance:
        Stand-off distance set point (m).  Disable distance control with < 0.
    max_linear_vel, max_angular_vel:
        Saturation limits.
    deadband:
        Image-plane error magnitude below which output is zeroed.
    """

    def __init__(
        self,
        yaw_kp: float = 0.8,
        yaw_ki: float = 0.0,
        yaw_kd: float = 0.05,
        heave_kp: float = 0.5,
        heave_ki: float = 0.0,
        heave_kd: float = 0.02,
        surge_kp: float = 0.3,
        surge_ki: float = 0.0,
        surge_kd: float = 0.01,
        desired_distance: float = 2.0,
        max_linear_vel: float = 0.5,
        max_angular_vel: float = 0.5,
        deadband: float = 0.05,
    ) -> None:
        super().__init__(
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
        )
        self._desired_dist = desired_distance
        self._deadband = deadband

        self._yaw_pid = _PID(yaw_kp, yaw_ki, yaw_kd, output_limit=max_angular_vel)
        self._heave_pid = _PID(heave_kp, heave_ki, heave_kd, output_limit=max_linear_vel)
        self._surge_pid = _PID(surge_kp, surge_ki, surge_kd, output_limit=max_linear_vel)

    # ------------------------------------------------------------------
    # ControllerBase interface
    # ------------------------------------------------------------------

    def compute_command(
        self,
        observation: TargetObservation,
        dt: float,
    ) -> ControlCommand:
        if not observation.is_valid:
            self.reset()
            return ControlCommand()

        ex = observation.image_x   # horizontal error → yaw
        ey = observation.image_y   # vertical   error → heave

        error_magnitude = (ex**2 + ey**2) ** 0.5
        if error_magnitude < self._deadband:
            wz = 0.0
            vz = 0.0
        else:
            wz = -self._yaw_pid.step(ex, dt)
            vz = -self._heave_pid.step(ey, dt)

        # Surge: close / open distance to the target
        vx = 0.0
        if observation.estimated_distance > 0 and self._desired_dist > 0:
            dist_error = observation.estimated_distance - self._desired_dist
            vx = self._surge_pid.step(dist_error, dt)

        cmd = ControlCommand(vx=vx, vy=0.0, vz=vz, wx=0.0, wy=0.0, wz=wz)
        return self._saturate(cmd)

    def reset(self) -> None:
        self._yaw_pid.reset()
        self._heave_pid.reset()
        self._surge_pid.reset()
