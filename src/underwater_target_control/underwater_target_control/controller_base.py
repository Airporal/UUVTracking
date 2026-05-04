"""Abstract base class for target-following controllers.

All controller backends (IBVS, PID, MPC, …) must subclass ``ControllerBase``
and implement ``compute_command()``.
"""

from __future__ import annotations

import abc
from dataclasses import dataclass
from typing import Optional


@dataclass
class ControlCommand:
    """Output of a single controller step."""

    # Linear velocities (m/s)
    vx: float = 0.0  # surge (forward/backward)
    vy: float = 0.0  # sway  (left/right)
    vz: float = 0.0  # heave (up/down)

    # Angular velocities (rad/s)
    wx: float = 0.0  # roll  rate
    wy: float = 0.0  # pitch rate
    wz: float = 0.0  # yaw   rate


@dataclass
class TargetObservation:
    """Simplified view of the current target state seen by the controller."""

    # Normalised image coords of the target centre [-1, 1]
    image_x: float = 0.0
    image_y: float = 0.0

    # Optional estimated distance (m); < 0 means unknown
    estimated_distance: float = -1.0

    # Whether a valid detection is available this step
    is_valid: bool = False


class ControllerBase(abc.ABC):
    """Abstract interface for target-following controllers."""

    def __init__(
        self,
        max_linear_vel: float = 0.5,
        max_angular_vel: float = 0.5,
    ) -> None:
        self._max_linear_vel = max_linear_vel
        self._max_angular_vel = max_angular_vel

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abc.abstractmethod
    def compute_command(
        self,
        observation: TargetObservation,
        dt: float,
    ) -> ControlCommand:
        """Compute and return the next motion command.

        Parameters
        ----------
        observation:
            Current target observation (image coords + validity flag).
        dt:
            Time since the last call (seconds).
        """

    @abc.abstractmethod
    def reset(self) -> None:
        """Reset any internal controller state (integrators, history, …)."""

    # ------------------------------------------------------------------
    # Helpers available to all subclasses
    # ------------------------------------------------------------------

    def _saturate(self, command: ControlCommand) -> ControlCommand:
        """Clamp command velocities to the configured limits."""
        mlv = self._max_linear_vel
        mav = self._max_angular_vel

        def _clamp(v: float, limit: float) -> float:
            return max(-limit, min(limit, v))

        return ControlCommand(
            vx=_clamp(command.vx, mlv),
            vy=_clamp(command.vy, mlv),
            vz=_clamp(command.vz, mlv),
            wx=_clamp(command.wx, mav),
            wy=_clamp(command.wy, mav),
            wz=_clamp(command.wz, mav),
        )

    @property
    def max_linear_vel(self) -> float:
        return self._max_linear_vel

    @max_linear_vel.setter
    def max_linear_vel(self, value: float) -> None:
        if value < 0:
            raise ValueError("max_linear_vel must be non-negative")
        self._max_linear_vel = value

    @property
    def max_angular_vel(self) -> float:
        return self._max_angular_vel

    @max_angular_vel.setter
    def max_angular_vel(self, value: float) -> None:
        if value < 0:
            raise ValueError("max_angular_vel must be non-negative")
        self._max_angular_vel = value
