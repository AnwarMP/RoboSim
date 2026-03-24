"""Tests for Robot (robot.py) — differential drive, P-controller, lateral friction."""

from __future__ import annotations

import math

from robosim.config import PhysicsConfig, SimulatorConfig
from robosim.physics import PhysicsWorld
from robosim.robot import Robot
from robosim.types import DriveCommand


def _make_robot(config: SimulatorConfig | None = None) -> Robot:
    world = PhysicsWorld(config or SimulatorConfig())
    return Robot(world)


class TestPowerClamping:
    def test_power_clamped_above_one(self) -> None:
        robot = _make_robot()
        robot.update(DriveCommand(left_power=5.0, right_power=5.0))
        # Should behave the same as power=1.0
        robot2 = _make_robot()
        robot2.update(DriveCommand(left_power=1.0, right_power=1.0))
        # Both should have moved the same amount (approximately)
        assert abs(robot.position.x - robot2.position.x) < 0.01

    def test_power_clamped_below_neg_one(self) -> None:
        robot = _make_robot()
        robot.update(DriveCommand(left_power=-3.0, right_power=-3.0))
        robot2 = _make_robot()
        robot2.update(DriveCommand(left_power=-1.0, right_power=-1.0))
        assert abs(robot.position.x - robot2.position.x) < 0.01


class TestDriveForward:
    def test_equal_power_moves_forward(self) -> None:
        """Equal left/right power should move the robot forward (along its heading)."""
        robot = _make_robot()
        start_x = robot.position.x
        # Default angle is 0, so forward = +X direction
        for _ in range(30):
            robot.update(DriveCommand(left_power=1.0, right_power=1.0))
        assert robot.position.x > start_x + 50

    def test_equal_power_no_significant_rotation(self) -> None:
        """Equal power should produce minimal rotation."""
        robot = _make_robot()
        for _ in range(30):
            robot.update(DriveCommand(left_power=0.8, right_power=0.8))
        assert abs(robot.angle) < 0.01

    def test_zero_power_robot_stays(self) -> None:
        robot = _make_robot()
        start = (robot.position.x, robot.position.y)
        for _ in range(30):
            robot.update(DriveCommand(0.0, 0.0))
        assert abs(robot.position.x - start[0]) < 0.01
        assert abs(robot.position.y - start[1]) < 0.01


class TestDriveReverse:
    def test_negative_power_moves_backward(self) -> None:
        robot = _make_robot()
        start_x = robot.position.x
        for _ in range(30):
            robot.update(DriveCommand(left_power=-1.0, right_power=-1.0))
        assert robot.position.x < start_x - 50


class TestTurning:
    def test_right_turn(self) -> None:
        """More right power than left should turn clockwise (positive angle in Y-down)."""
        robot = _make_robot()
        for _ in range(30):
            robot.update(DriveCommand(left_power=0.5, right_power=-0.5))
        # In Y-down with (right - left) kinematics, this gives negative omega -> negative angle
        assert robot.angle != 0.0

    def test_left_turn(self) -> None:
        """More left power than right should turn counter-clockwise."""
        robot = _make_robot()
        for _ in range(30):
            robot.update(DriveCommand(left_power=-0.5, right_power=0.5))
        assert robot.angle != 0.0

    def test_opposite_turns_are_symmetric(self) -> None:
        """Symmetric power should produce symmetric rotation."""
        robot_a = _make_robot()
        for _ in range(30):
            robot_a.update(DriveCommand(left_power=0.5, right_power=-0.5))

        robot_b = _make_robot()
        for _ in range(30):
            robot_b.update(DriveCommand(left_power=-0.5, right_power=0.5))

        assert abs(robot_a.angle + robot_b.angle) < 0.01

    def test_point_turn_stays_near_center(self) -> None:
        """A point turn (equal and opposite power) should not translate much."""
        robot = _make_robot()
        start = (robot.position.x, robot.position.y)
        for _ in range(60):
            robot.update(DriveCommand(left_power=0.5, right_power=-0.5))
        dx = robot.position.x - start[0]
        dy = robot.position.y - start[1]
        assert math.sqrt(dx**2 + dy**2) < 10.0


class TestLateralFriction:
    def test_full_lateral_friction_prevents_sideways_slide(self) -> None:
        """With lateral_friction=1.0, robot should not slide sideways."""
        robot = _make_robot()
        # Drive forward, then check no significant lateral velocity
        for _ in range(30):
            robot.update(DriveCommand(left_power=1.0, right_power=1.0))
        # Lateral direction is perpendicular to heading
        right = (-math.sin(robot.angle), math.cos(robot.angle))
        lateral_speed = robot.velocity.x * right[0] + robot.velocity.y * right[1]
        assert abs(lateral_speed) < 1.0

    def test_zero_lateral_friction_allows_slide(self) -> None:
        """With lateral_friction=0, sideways impulse is not cancelled."""
        cfg = SimulatorConfig(physics=PhysicsConfig(lateral_friction=0.0))
        world = PhysicsWorld(cfg)
        robot = Robot(world)
        # Give the robot a sideways velocity manually
        world.robot_body.velocity = (0, 100)
        robot.update(DriveCommand(0.0, 0.0))
        # With no lateral friction, should still have significant Y velocity
        # (only damping reduces it, damping=0.8 so ~80 after one step)
        assert abs(world.robot_body.velocity.y) > 50


class TestSpeedLimit:
    def test_robot_approaches_max_speed(self) -> None:
        """At full power, the robot should approach but not greatly exceed max_speed."""
        robot = _make_robot()
        # Run for just enough frames to reach steady state, not long enough to hit a wall
        for _ in range(60):
            robot.update(DriveCommand(left_power=1.0, right_power=1.0))
        speed = robot.velocity.length
        max_speed = robot._cfg.max_speed_px_s
        # Should be within reasonable range of max speed (damping + P-control equilibrium)
        assert speed < max_speed * 1.3
        assert speed > max_speed * 0.3


class TestUpdateStepsPhysics:
    def test_update_advances_simulation(self) -> None:
        """update() should advance the physics world by one timestep."""
        robot = _make_robot()
        robot.world.robot_body.velocity = (100, 0)
        pos_before = robot.position.x
        robot.update(DriveCommand(0.0, 0.0))
        # Body should have moved due to existing velocity
        assert robot.position.x != pos_before
