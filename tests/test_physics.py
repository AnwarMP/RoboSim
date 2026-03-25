"""Tests for PhysicsWorld (physics.py) — space setup, robot body, stepping."""

import math

import pymunk

from robosim.config import COLLISION_ROBOT, PhysicsConfig, SimulatorConfig, StartConfig
from robosim.physics import PhysicsWorld


def _make_world(config: SimulatorConfig | None = None) -> PhysicsWorld:
    return PhysicsWorld(config or SimulatorConfig())


class TestSpaceSetup:
    def test_gravity_is_zero(self) -> None:
        world = _make_world()
        assert world.space.gravity == (0, 0)

    def test_damping(self) -> None:
        world = _make_world()
        assert world.space.damping == 0.8

    def test_arena_walls_in_space(self) -> None:
        world = _make_world()
        assert len(world.arena.walls) == 4


class TestRobotBody:
    def test_robot_at_arena_center(self) -> None:
        world = _make_world()
        center = world.arena_cfg.arena_size_px / 2.0
        assert world.robot_body.position.x == center
        assert world.robot_body.position.y == center

    def test_robot_mass(self) -> None:
        world = _make_world()
        assert world.robot_body.mass == 5.0

    def test_robot_moment_auto_calculated(self) -> None:
        world = _make_world()
        # Moment should be positive and auto-calculated for box shape
        assert world.robot_body.moment > 0

    def test_robot_shape_properties(self) -> None:
        world = _make_world()
        assert world.robot_shape.friction == 0.7
        assert world.robot_shape.elasticity == 0.4
        assert world.robot_shape.collision_type == COLLISION_ROBOT

    def test_robot_is_dynamic(self) -> None:
        world = _make_world()
        assert world.robot_body.body_type == pymunk.Body.DYNAMIC

    def test_custom_start_position(self) -> None:
        cfg = SimulatorConfig(start=StartConfig(x=100.0, y=200.0))
        world = _make_world(cfg)
        assert world.robot_body.position.x == 100.0
        assert world.robot_body.position.y == 200.0

    def test_custom_start_heading(self) -> None:
        cfg = SimulatorConfig(start=StartConfig(heading_deg=90.0))
        world = _make_world(cfg)
        assert abs(world.robot_body.angle - math.radians(90.0)) < 1e-9

    def test_default_start_is_arena_center(self) -> None:
        world = _make_world()
        center = world.arena_cfg.arena_size_px / 2.0
        assert world.robot_body.position.x == center
        assert world.robot_body.position.y == center
        assert world.robot_body.angle == 0.0


class TestTorqueGain:
    def test_auto_torque_gain(self) -> None:
        world = _make_world()
        expected = 10.0 * world.robot_body.moment
        assert world.torque_gain == expected

    def test_explicit_torque_gain(self) -> None:
        cfg = SimulatorConfig(physics=PhysicsConfig(torque_gain=42.0))
        world = _make_world(cfg)
        assert world.torque_gain == 42.0


class TestStepping:
    def test_step_does_not_raise(self) -> None:
        world = _make_world()
        world.step()

    def test_stationary_robot_stays_put(self) -> None:
        world = _make_world()
        start_pos = tuple(world.robot_body.position)
        for _ in range(60):
            world.step()
        end_pos = tuple(world.robot_body.position)
        assert start_pos[0] == end_pos[0]
        assert start_pos[1] == end_pos[1]

    def test_robot_moves_when_force_applied(self) -> None:
        world = _make_world()
        start_x = world.robot_body.position.x
        # Apply a rightward force for several frames
        for _ in range(30):
            world.robot_body.apply_force_at_world_point((500, 0), world.robot_body.position)
            world.step()
        assert world.robot_body.position.x > start_x

    def test_robot_stops_at_wall(self) -> None:
        """Push the robot toward a wall; it should not escape the arena."""
        world = _make_world()
        arena_size = world.arena_cfg.arena_size_px
        # Push hard to the right for a long time
        for _ in range(300):
            world.robot_body.apply_force_at_world_point((5000, 0), world.robot_body.position)
            world.step()
        # Robot x should not exceed the arena boundary (with some tolerance for wall radius)
        assert world.robot_body.position.x < arena_size

    def test_damping_slows_robot(self) -> None:
        """A robot given an initial velocity should decelerate due to damping."""
        world = _make_world()
        world.robot_body.velocity = (100, 0)
        world.step()
        # After one step with damping=0.8, velocity should be reduced
        assert abs(world.robot_body.velocity.x) < 100
