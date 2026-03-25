"""Sensor models and noise pipeline.

Currently implements: wheel encoders, IMU.
Rangefinders will be added in a subsequent PR.
"""

from __future__ import annotations

import math
import random

from pymunk import Vec2d

from robosim.config import NoiseConfig, PhysicsConfig


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


class NoisePipeline:
    """Shared two-stage noise model used by all sensors.

    Each call to :meth:`apply` adds:

    1. Instantaneous Gaussian noise: ``gauss(0, σ_noise)``
    2. Accumulated bias drift: a random walk that grows over time

    The bias drift step is scaled by ``√dt`` so that noise statistics are
    independent of the simulation timestep.

    With default ``NoiseConfig`` (all zeros) this is a pass-through.
    """

    def __init__(self, config: NoiseConfig) -> None:
        self._gaussian_sigma = config.gaussian_sigma
        self._drift_sigma = config.bias_drift_sigma
        self._accumulated_bias: float = 0.0

    @property
    def accumulated_bias(self) -> float:
        return self._accumulated_bias

    def apply(self, true_value: float, dt: float) -> float:
        """Return a noisy reading from *true_value*."""
        # Update bias random walk
        if self._drift_sigma:
            self._accumulated_bias += random.gauss(0.0, self._drift_sigma * math.sqrt(dt))

        noise = 0.0
        if self._gaussian_sigma:
            noise = random.gauss(0.0, self._gaussian_sigma)

        return true_value + noise + self._accumulated_bias

    def reset(self) -> None:
        self._accumulated_bias = 0.0


class IMU:
    """Inertial measurement unit — heading and angular velocity.

    Heading is measured relative to the robot's initial angle at sim start,
    reported in degrees with counter-clockwise positive.

    Noise is applied to the angular velocity reading.  Heading drift is the
    *integrated* effect of angular velocity noise over time — this mimics
    how real MEMS gyroscopes accumulate error.
    """

    def __init__(self, initial_angle_rad: float, noise_cfg: NoiseConfig) -> None:
        self._initial_angle = initial_angle_rad
        self._noise = NoisePipeline(noise_cfg)
        # Heading is integrated from noisy angular velocity
        self._integrated_heading_deg: float = 0.0
        self._has_noise = noise_cfg.gaussian_sigma != 0.0 or noise_cfg.bias_drift_sigma != 0.0

    def update(self, body_angle_rad: float, body_angular_vel_rad: float, dt: float) -> None:
        """Read the body state and update IMU readings.

        Parameters
        ----------
        body_angle_rad:
            Current Pymunk body angle in radians.
        body_angular_vel_rad:
            Current angular velocity in rad/s.
        dt:
            Timestep in seconds.
        """
        true_angular_vel_deg = math.degrees(body_angular_vel_rad)

        if self._has_noise:
            # Apply noise to angular velocity, then integrate for heading
            self._noisy_angular_vel_deg = self._noise.apply(true_angular_vel_deg, dt)
            self._integrated_heading_deg += self._noisy_angular_vel_deg * dt
        else:
            # Perfect sensors — derive heading directly from body angle
            self._noisy_angular_vel_deg = true_angular_vel_deg
            self._integrated_heading_deg = math.degrees(body_angle_rad - self._initial_angle)

    @property
    def heading_deg(self) -> float:
        """Current heading in degrees relative to start (CCW positive)."""
        return self._integrated_heading_deg

    @property
    def angular_vel_deg(self) -> float:
        """Current angular velocity in degrees/second."""
        return self._noisy_angular_vel_deg if hasattr(self, "_noisy_angular_vel_deg") else 0.0

    def reset(self, initial_angle_rad: float) -> None:
        self._initial_angle = initial_angle_rad
        self._noise.reset()
        self._integrated_heading_deg = 0.0
        self._noisy_angular_vel_deg = 0.0
