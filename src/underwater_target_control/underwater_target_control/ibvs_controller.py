"""Image-Based Visual Servoing (IBVS) controller.

This controller drives the UUV to keep the target centred in the image frame.
It treats the normalised image-plane error as its feature vector and generates
proportional velocity commands.

Control law (simplified):
    e = [image_x, image_y]          (feature error)
    u = -λ · L⁺ · e
where L⁺ is a simplified pseudo-inverse of the image Jacobian and λ is the
visual-servo gain.

For a UUV, the mapping is:
    image_x  → yaw   rate (wz)
    image_y  → pitch rate (wy) / heave (vz)
    distance → surge (vx)
"""

from __future__ import annotations

from underwater_target_control.controller_base import (
    ControlCommand,
    ControllerBase,
    TargetObservation,
)


class IBVSController(ControllerBase):
    """Simplified Image-Based Visual Servoing controller.

    Parameters
    ----------
    lambda_gain:
        Visual servoing gain (proportional).
    desired_image_x, desired_image_y:
        Desired normalised target position in the image (default: centre).
    desired_distance:
        Desired stand-off distance from the target (m).  Set < 0 to disable
        distance control.
    max_linear_vel, max_angular_vel:
        Saturation limits (m/s, rad/s).
    deadband:
        Image-plane error magnitude below which commands are zeroed.
    """

    def __init__(
        self,
        lambda_gain: float = 0.5,
        desired_image_x: float = 0.0,
        desired_image_y: float = 0.0,
        desired_distance: float = 2.0,
        distance_gain: float = 0.3,
        max_linear_vel: float = 0.5,
        max_angular_vel: float = 0.5,
        deadband: float = 0.05,
    ) -> None:
        super().__init__(
            max_linear_vel=max_linear_vel,
            max_angular_vel=max_angular_vel,
        )
        self._lambda = lambda_gain
        self._desired_x = desired_image_x
        self._desired_y = desired_image_y
        self._desired_dist = desired_distance
        self._dist_gain = distance_gain
        self._deadband = deadband

    # ------------------------------------------------------------------
    # ControllerBase interface
    # ------------------------------------------------------------------

    def compute_command(
        self,
        observation: TargetObservation,
        dt: float,
    ) -> ControlCommand:
        if not observation.is_valid:
            return ControlCommand()  # zero command when no target

        ex = observation.image_x - self._desired_x
        ey = observation.image_y - self._desired_y

        error_magnitude = (ex**2 + ey**2) ** 0.5
        if error_magnitude < self._deadband:
            wz = 0.0
            vy = 0.0
        else:
            # Yaw rate to centre horizontally; pitch/heave to centre vertically
            wz = -self._lambda * ex
            vy = -self._lambda * ey

        # Surge: drive toward / away from target based on distance error
        vx = 0.0
        if observation.estimated_distance > 0 and self._desired_dist > 0:
            dist_error = observation.estimated_distance - self._desired_dist
            vx = self._dist_gain * dist_error

        cmd = ControlCommand(vx=vx, vy=vy, vz=0.0, wx=0.0, wy=0.0, wz=wz)
        return self._saturate(cmd)

    def reset(self) -> None:
        pass  # IBVS is stateless; nothing to reset
