"""Tests for the encoder sensor models."""

from __future__ import annotations

import math

from pymunk import Vec2d

from robosim.config import NoiseConfig, PhysicsConfig
from robosim.sensors import IMU, EncoderPair, NoisePipeline, WheelEncoder


class TestWheelEncoder:
    """Unit tests for a single WheelEncoder."""

    def test_starts_at_zero(self) -> None:
        enc = WheelEncoder()
        assert enc.ticks == 0

    def test_forward_accumulates_positive_ticks(self) -> None:
        enc = WheelEncoder()
        # Drive forward at 100 px/s for 1 second (60 steps at 1/60)
        dt = 1.0 / 60.0
        for _ in range(60):
            enc.update(100.0, dt)
        assert enc.ticks > 0

    def test_backward_accumulates_negative_ticks(self) -> None:
        enc = WheelEncoder()
        dt = 1.0 / 60.0
        for _ in range(60):
            enc.update(-100.0, dt)
        assert enc.ticks < 0

    def test_stationary_no_change(self) -> None:
        enc = WheelEncoder()
        dt = 1.0 / 60.0
        for _ in range(100):
            enc.update(0.0, dt)
        assert enc.ticks == 0

    def test_reset(self) -> None:
        enc = WheelEncoder()
        enc.update(200.0, 1.0)
        assert enc.ticks != 0
        enc.reset()
        assert enc.ticks == 0

    def test_ticks_proportional_to_speed(self) -> None:
        """Doubling speed should roughly double ticks over the same duration."""
        enc_slow = WheelEncoder()
        enc_fast = WheelEncoder()
        dt = 1.0 / 60.0
        for _ in range(60):
            enc_slow.update(50.0, dt)
            enc_fast.update(100.0, dt)
        ratio = enc_fast.ticks / enc_slow.ticks
        assert 1.9 < ratio < 2.1

    def test_known_value_one_revolution(self) -> None:
        """One full revolution of a unit-radius wheel = 2π px of travel.

        After 2π px at 1 px/s for 2π seconds, we should see ~ticks_per_rev ticks.
        """
        enc = WheelEncoder(ticks_per_rev=100.0)
        duration = 2.0 * math.pi
        dt = 1.0 / 60.0
        steps = int(duration / dt)
        for _ in range(steps):
            enc.update(1.0, dt)
        # Should be close to 100 ticks (one full revolution)
        assert abs(enc.ticks - 100) <= 2  # allow small rounding


class TestEncoderPair:
    """Tests for the paired left/right encoder system."""

    def test_straight_forward_both_increase(self) -> None:
        cfg = PhysicsConfig()
        pair = EncoderPair(cfg)
        dt = cfg.timestep
        # Robot facing right (angle=0), moving right at 100 px/s, no rotation
        vel = Vec2d(100.0, 0.0)
        for _ in range(60):
            pair.update(vel, 0.0, 0.0, dt)
        assert pair.enc_left > 0
        assert pair.enc_right > 0

    def test_straight_forward_symmetric(self) -> None:
        """Straight line motion should give equal ticks on both sides."""
        cfg = PhysicsConfig()
        pair = EncoderPair(cfg)
        dt = cfg.timestep
        vel = Vec2d(100.0, 0.0)
        for _ in range(120):
            pair.update(vel, 0.0, 0.0, dt)
        assert pair.enc_left == pair.enc_right

    def test_pure_rotation_opposite_signs(self) -> None:
        """Rotating in place: one encoder goes forward, the other backward."""
        cfg = PhysicsConfig()
        pair = EncoderPair(cfg)
        dt = cfg.timestep
        # Stationary (no linear velocity), spinning CCW at 1 rad/s
        vel = Vec2d(0.0, 0.0)
        for _ in range(120):
            pair.update(vel, 0.0, 1.0, dt)
        # CCW rotation: left wheel goes backward, right goes forward
        assert pair.enc_left < 0
        assert pair.enc_right > 0

    def test_pure_rotation_magnitudes_equal(self) -> None:
        """In pure rotation, left and right magnitudes should match."""
        cfg = PhysicsConfig()
        pair = EncoderPair(cfg)
        dt = cfg.timestep
        vel = Vec2d(0.0, 0.0)
        for _ in range(120):
            pair.update(vel, 0.0, 2.0, dt)
        assert abs(abs(pair.enc_left) - abs(pair.enc_right)) <= 1

    def test_angled_body_projects_correctly(self) -> None:
        """Velocity must be projected into body frame, not just use x-component."""
        cfg = PhysicsConfig()
        pair = EncoderPair(cfg)
        dt = cfg.timestep
        # Robot facing up-right at 45°, velocity also at 45° (moving forward)
        angle = math.pi / 4
        speed = 100.0
        vel = Vec2d(speed * math.cos(angle), speed * math.sin(angle))
        for _ in range(60):
            pair.update(vel, angle, 0.0, dt)
        assert pair.enc_left > 0
        assert pair.enc_right > 0

    def test_sideways_velocity_no_ticks(self) -> None:
        """Pure lateral motion should produce zero forward ticks."""
        cfg = PhysicsConfig()
        pair = EncoderPair(cfg)
        dt = cfg.timestep
        # Robot facing right (angle=0), but velocity is purely upward (sideways)
        vel = Vec2d(0.0, 100.0)
        for _ in range(60):
            pair.update(vel, 0.0, 0.0, dt)
        assert pair.enc_left == 0
        assert pair.enc_right == 0

    def test_reset(self) -> None:
        cfg = PhysicsConfig()
        pair = EncoderPair(cfg)
        pair.update(Vec2d(100.0, 0.0), 0.0, 0.0, 1.0)
        assert pair.enc_left != 0
        pair.reset()
        assert pair.enc_left == 0
        assert pair.enc_right == 0


class TestNoisePipeline:
    """Unit tests for the shared noise model."""

    def test_zero_noise_is_passthrough(self) -> None:
        pipe = NoisePipeline(NoiseConfig())
        assert pipe.apply(42.0, 1.0 / 60) == 42.0
        assert pipe.apply(-3.5, 1.0 / 60) == -3.5

    def test_zero_noise_no_bias_drift(self) -> None:
        pipe = NoisePipeline(NoiseConfig())
        for _ in range(1000):
            pipe.apply(0.0, 1.0 / 60)
        assert pipe.accumulated_bias == 0.0

    def test_gaussian_noise_has_nonzero_variance(self) -> None:
        import random as _rng

        _rng.seed(12345)
        pipe = NoisePipeline(NoiseConfig(gaussian_sigma=1.0))
        readings = [pipe.apply(0.0, 1.0 / 60) for _ in range(500)]
        # With σ=1, readings should scatter around 0
        assert max(readings) > 0.5
        assert min(readings) < -0.5

    def test_bias_drift_accumulates_over_time(self) -> None:
        import random as _rng

        _rng.seed(99)
        pipe = NoisePipeline(NoiseConfig(bias_drift_sigma=0.1))
        for _ in range(600):
            pipe.apply(0.0, 1.0 / 60)
        # After 10 simulated seconds of drift, bias should be nonzero
        assert pipe.accumulated_bias != 0.0

    def test_reset_clears_bias(self) -> None:
        import random as _rng

        _rng.seed(42)
        pipe = NoisePipeline(NoiseConfig(bias_drift_sigma=0.5))
        for _ in range(100):
            pipe.apply(0.0, 1.0 / 60)
        assert pipe.accumulated_bias != 0.0
        pipe.reset()
        assert pipe.accumulated_bias == 0.0


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
