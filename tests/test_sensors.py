"""Tests for the encoder sensor models."""

from __future__ import annotations

import math

from pymunk import Vec2d

from robosim.config import PhysicsConfig
from robosim.sensors import DEFAULT_TICKS_PER_REV, EncoderPair, WheelEncoder


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
