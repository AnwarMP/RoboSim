"""Sensor models and noise pipeline.

Currently implements: wheel encoders.
IMU and rangefinders will be added in subsequent PRs.
"""

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
        # Convert linear distance (px) travelled this frame to wheel revolutions,
        # then to encoder ticks.
        #
        # wheel_circumference is implicit — we treat "one revolution" as the
        # distance the wheel travels in one full rotation.  Since wheel radius
        # isn't specified separately, we use: distance = speed × dt, and
        # angular_distance = distance / (wheel_base / 2).  But the PRD defines:
        #   ticks += angular_velocity × dt × ticks_per_rev / (2π)
        # Rewritten with linear speed:
        #   angular_vel = linear_speed / wheel_radius
        # We use wheel_base/2 as a proxy for wheel_radius.
        #
        # However, a simpler and equivalent approach that the PRD suggests:
        # Just convert linear travel to ticks via a fixed ratio.
        # ticks_per_px = ticks_per_rev / (2π × wheel_radius)
        # For now, we use the direct integration:
        #   distance_px = wheel_speed * dt
        #   revolutions = distance_px / (π × wheel_diameter_approx)
        #
        # Simplification: since ticks_per_rev is configurable and acts as
        # a scaling constant, we treat 1 revolution = 2π px of travel at
        # unit wheel radius.  The user tunes ticks_per_rev to match their
        # desired ticks-per-metre.
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
        """Read the body state and advance both encoders one tick.

        Parameters
        ----------
        body_velocity:
            World-frame velocity vector of the robot body (px/s).
        body_angle:
            Current heading of the robot body in radians.
        angular_velocity:
            Current angular velocity in rad/s.
        dt:
            Timestep in seconds.
        """
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
