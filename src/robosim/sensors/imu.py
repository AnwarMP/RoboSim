"""Inertial measurement unit — heading and angular velocity."""

from __future__ import annotations

import math

from robosim.config import NoiseConfig
from robosim.sensors.noise import NoisePipeline


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
