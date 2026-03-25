"""Tests for the IMU sensor."""

from __future__ import annotations

import math

from robosim.config import NoiseConfig
from robosim.sensors import IMU


class TestIMU:
    """Tests for the IMU sensor."""

    def test_starts_at_zero_heading(self) -> None:
        imu = IMU(0.0, NoiseConfig())
        imu.update(0.0, 0.0, 1.0 / 60)
        assert imu.heading_deg == 0.0

    def test_heading_relative_to_initial_angle(self) -> None:
        """If robot starts at 1 rad and is now at 1 rad, heading should be 0."""
        imu = IMU(1.0, NoiseConfig())
        imu.update(1.0, 0.0, 1.0 / 60)
        assert abs(imu.heading_deg) < 1e-9

    def test_90_degree_turn(self) -> None:
        """Turning π/2 from initial angle should give ~90°."""
        initial = 0.0
        imu = IMU(initial, NoiseConfig())
        imu.update(math.pi / 2, 0.0, 1.0 / 60)
        assert abs(imu.heading_deg - 90.0) < 1e-6

    def test_negative_turn(self) -> None:
        """Turning -π/2 should give ~-90°."""
        imu = IMU(0.0, NoiseConfig())
        imu.update(-math.pi / 2, 0.0, 1.0 / 60)
        assert abs(imu.heading_deg - (-90.0)) < 1e-6

    def test_angular_velocity_conversion(self) -> None:
        """1 rad/s should report ~57.3°/s with zero noise."""
        imu = IMU(0.0, NoiseConfig())
        imu.update(0.0, 1.0, 1.0 / 60)
        assert abs(imu.angular_vel_deg - math.degrees(1.0)) < 1e-6

    def test_zero_angular_velocity_when_stationary(self) -> None:
        imu = IMU(0.0, NoiseConfig())
        imu.update(0.0, 0.0, 1.0 / 60)
        assert imu.angular_vel_deg == 0.0

    def test_noise_causes_heading_drift(self) -> None:
        """With noise enabled, heading should drift from the true value over time."""
        import random as _rng

        _rng.seed(777)
        noisy_cfg = NoiseConfig(gaussian_sigma=0.01, bias_drift_sigma=0.001)
        imu = IMU(0.0, noisy_cfg)
        # Simulate 10 seconds of no rotation — true heading stays at 0
        dt = 1.0 / 60
        for _ in range(600):
            imu.update(0.0, 0.0, dt)
        # With noise, heading should have drifted away from 0
        assert abs(imu.heading_deg) > 0.001

    def test_noisy_angular_velocity_differs_from_true(self) -> None:
        """With gaussian noise, angular velocity reading should deviate from truth."""
        import random as _rng

        _rng.seed(555)
        noisy_cfg = NoiseConfig(gaussian_sigma=1.0)
        imu = IMU(0.0, noisy_cfg)
        imu.update(0.0, 0.0, 1.0 / 60)
        # True angular vel is 0, but noise should make it nonzero
        assert imu.angular_vel_deg != 0.0

    def test_reset(self) -> None:
        imu = IMU(0.0, NoiseConfig())
        imu.update(math.pi, 2.0, 1.0 / 60)
        assert imu.heading_deg != 0.0
        imu.reset(math.pi)
        assert imu.heading_deg == 0.0
        assert imu.angular_vel_deg == 0.0

    def test_perfect_heading_tracks_body_angle(self) -> None:
        """With zero noise, heading should exactly track body angle changes."""
        imu = IMU(0.0, NoiseConfig())
        dt = 1.0 / 60
        # Simulate gradual rotation
        for step in range(60):
            angle = (step + 1) * 0.01  # slowly increasing angle
            imu.update(angle, 0.01 / dt, dt)
        expected_deg = math.degrees(60 * 0.01)
        assert abs(imu.heading_deg - expected_deg) < 1e-6
