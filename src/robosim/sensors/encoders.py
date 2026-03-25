"""Wheel encoder sensor models."""

from __future__ import annotations

import math

from pymunk import Vec2d

from robosim.config import PhysicsConfig


# Default ticks per full wheel revolution (geared DC motor)
DEFAULT_TICKS_PER_REV: float = 537.7


class WheelEncoder:
    """Cumulative tick counter for one wheel.

    Each frame the encoder integrates the wheel's linear speed into an
    internal float accumulator.  The public reading is ``int(accumulator)``,
    which naturally truncates fractional ticks — matching how real encoders
    work (you only get a tick when the slot fully passes the sensor).

    Ticks are derived from the *actual* post-physics body velocity, not
    the commanded power.  If the robot stalls against a wall the encoders
    reflect the reduced (or zero) motion.
    """

    def __init__(self, ticks_per_rev: float = DEFAULT_TICKS_PER_REV) -> None:
        self.ticks_per_rev = ticks_per_rev
        self._accumulator: float = 0.0

    @property
    def ticks(self) -> int:
        """Current cumulative tick count (truncated to int)."""
        return int(self._accumulator)

    def update(self, wheel_speed_px_s: float, dt: float) -> None:
        """Advance the accumulator by one timestep.

        Parameters
        ----------
        wheel_speed_px_s:
            Linear speed of this wheel in px/s (positive = forward).
        dt:
            Timestep duration in seconds.
        """
        distance_px = wheel_speed_px_s * dt
        # revolutions at unit-radius wheel
        revolutions = distance_px / (2.0 * math.pi)
        self._accumulator += revolutions * self.ticks_per_rev

    def reset(self) -> None:
        self._accumulator = 0.0


class EncoderPair:
    """Left + right wheel encoders for a differential-drive robot.

    Computes per-wheel linear speeds from the body's forward velocity
    and angular velocity using differential-drive kinematics::

        v_left  = v_forward − ω × (wheel_base / 2)
        v_right = v_forward + ω × (wheel_base / 2)
    """

    def __init__(
        self,
        physics_cfg: PhysicsConfig,
        ticks_per_rev: float = DEFAULT_TICKS_PER_REV,
    ) -> None:
        self._half_base = physics_cfg.wheel_base_px / 2.0
        self.left = WheelEncoder(ticks_per_rev)
        self.right = WheelEncoder(ticks_per_rev)

    def update(
        self,
        body_velocity: Vec2d,
        body_angle: float,
        angular_velocity: float,
        dt: float,
    ) -> None:
        """Read the body state and advance both encoders one tick."""
        # Project world velocity into body-forward speed
        forward = Vec2d(math.cos(body_angle), math.sin(body_angle))
        v_forward = body_velocity.dot(forward)

        # Differential drive: per-wheel speeds
        v_left = v_forward - angular_velocity * self._half_base
        v_right = v_forward + angular_velocity * self._half_base

        self.left.update(v_left, dt)
        self.right.update(v_right, dt)

    @property
    def enc_left(self) -> int:
        return self.left.ticks

    @property
    def enc_right(self) -> int:
        return self.right.ticks

    def reset(self) -> None:
        self.left.reset()
        self.right.reset()
