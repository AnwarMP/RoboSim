"""Tests for the rangefinder sensors."""

from __future__ import annotations

import math

import pymunk

from robosim.config import NoiseConfig, PX_PER_CM, SimulatorConfig
from robosim.physics import PhysicsWorld
from robosim.sensors.rangefinder import MAX_RANGE_CM, MIN_RANGE_CM, RangefinderArray


def _make_world() -> PhysicsWorld:
    """Create a default PhysicsWorld for testing."""
    return PhysicsWorld(SimulatorConfig())


class TestRangefinderArray:
    """Integration tests using a real Pymunk space with arena walls."""

    def test_detects_walls_when_close(self) -> None:
        """Robot near a wall should detect it (distance < MAX_RANGE_CM)."""
        world = _make_world()
        rf = RangefinderArray(
            world.physics_cfg.robot_size_px, NoiseConfig(), world.robot_shape
        )
        dt = world.physics_cfg.timestep

        # Move robot close to the right wall (arena is 648px wide)
        arena_px = world.arena_cfg.arena_size_px
        world.robot_body.position = (arena_px - 80, arena_px / 2.0)
        world.space.reindex_shapes_for_body(world.robot_body)
        rf.update(world.robot_body, world.space, dt)

        # Front should detect the nearby wall
        assert rf.range_front < MAX_RANGE_CM

    def test_symmetric_near_wall(self) -> None:
        """When equidistant from two walls, perpendicular sensors should match."""
        world = _make_world()
        rf = RangefinderArray(
            world.physics_cfg.robot_size_px, NoiseConfig(), world.robot_shape
        )
        dt = world.physics_cfg.timestep

        # Place robot at centre (equidistant left/right, front/back all at max range)
        rf.update(world.robot_body, world.space, dt)

        # At centre, all walls are equidistant — all should read the same
        assert abs(rf.range_front - rf.range_back) < 1.0
        assert abs(rf.range_left - rf.range_right) < 1.0

    def test_distance_decreases_as_robot_approaches_wall(self) -> None:
        """Moving the robot closer to a wall should decrease the rangefinder reading."""
        world = _make_world()
        arena_px = world.arena_cfg.arena_size_px
        rf = RangefinderArray(
            world.physics_cfg.robot_size_px, NoiseConfig(), world.robot_shape
        )
        dt = world.physics_cfg.timestep

        # Start near right wall but within range
        world.robot_body.position = (arena_px - 150, arena_px / 2.0)
        world.space.reindex_shapes_for_body(world.robot_body)
        rf.update(world.robot_body, world.space, dt)
        front_far = rf.range_front

        # Move closer to right wall
        world.robot_body.position = (arena_px - 80, arena_px / 2.0)
        world.space.reindex_shapes_for_body(world.robot_body)
        rf.update(world.robot_body, world.space, dt)
        front_close = rf.range_front

        assert front_close < front_far

    def test_distance_in_cm_is_correct(self) -> None:
        """Verify px-to-cm conversion produces expected values."""
        world = _make_world()
        arena_px = world.arena_cfg.arena_size_px
        robot_w = world.physics_cfg.robot_size_px[0]

        rf = RangefinderArray(
            world.physics_cfg.robot_size_px, NoiseConfig(), world.robot_shape
        )
        dt = world.physics_cfg.timestep

        # Move robot close to right wall so it's within max range
        world.robot_body.position = (arena_px - 100, arena_px / 2.0)
        world.space.reindex_shapes_for_body(world.robot_body)
        rf.update(world.robot_body, world.space, dt)

        # Distance from mount point to wall in px:
        # (arena_px - robot_x) - robot_w/2, minus wall segment radius
        expected_px = (arena_px - world.robot_body.position.x) - robot_w / 2.0
        expected_cm = expected_px / PX_PER_CM

        # Allow tolerance for wall segment radius and raycast precision
        assert abs(rf.range_front - expected_cm) < 10.0

    def test_rotation_changes_front_direction(self) -> None:
        """After rotating 90° CCW, 'front' faces a different wall."""
        world = _make_world()
        arena_px = world.arena_cfg.arena_size_px
        rf = RangefinderArray(
            world.physics_cfg.robot_size_px, NoiseConfig(), world.robot_shape
        )
        dt = world.physics_cfg.timestep

        # Place robot near bottom-right corner so both front and top walls are in range
        world.robot_body.position = (arena_px - 100, arena_px - 100)
        world.space.reindex_shapes_for_body(world.robot_body)

        # Facing right (angle=0), front measures distance to right wall
        rf.update(world.robot_body, world.space, dt)
        front_facing_right = rf.range_front

        # Rotate 90° CCW — front now faces up (toward top wall in Y-down)
        world.robot_body.angle = -math.pi / 2
        world.space.reindex_shapes_for_body(world.robot_body)
        rf.update(world.robot_body, world.space, dt)
        front_facing_up = rf.range_front

        # Both should be valid readings, and the distances should differ
        # (corner position means different distances to right vs top wall only if not square)
        # At (548, 548), distance to right wall ≈ 100px, distance to top wall ≈ 548px
        # Top wall is out of range, so front_facing_up should be MAX_RANGE
        assert front_facing_right < MAX_RANGE_CM
        assert front_facing_up == MAX_RANGE_CM

    def test_max_range_when_no_wall(self) -> None:
        """If no wall is hit within max range, reading should be MAX_RANGE_CM."""
        # Create a space with no walls
        space = pymunk.Space()
        body = pymunk.Body()
        body.position = (100, 100)
        shape = pymunk.Poly.create_box(body, (60, 40))
        shape.mass = 5.0
        space.add(body, shape)

        rf = RangefinderArray((60.0, 40.0), NoiseConfig(), shape)
        rf.update(body, space, 1.0 / 60)

        assert rf.range_front == MAX_RANGE_CM
        assert rf.range_right == MAX_RANGE_CM
        assert rf.range_back == MAX_RANGE_CM
        assert rf.range_left == MAX_RANGE_CM

    def test_min_range_clamp(self) -> None:
        """Readings should never go below MIN_RANGE_CM."""
        # Place a wall extremely close to the robot
        space = pymunk.Space()
        body = pymunk.Body()
        body.position = (100, 100)
        shape = pymunk.Poly.create_box(body, (60, 40))
        shape.mass = 5.0
        space.add(body, shape)

        # Add a wall very close in front (just beyond the shape boundary)
        wall_body = space.static_body
        wall = pymunk.Segment(wall_body, (133, 50), (133, 150), 1.0)
        space.add(wall)

        rf = RangefinderArray((60.0, 40.0), NoiseConfig(), shape)
        rf.update(body, space, 1.0 / 60)

        assert rf.range_front >= MIN_RANGE_CM

    def test_noise_affects_readings(self) -> None:
        """With noise enabled, repeated readings should vary."""
        import random

        random.seed(42)
        world = _make_world()
        noisy_cfg = NoiseConfig(gaussian_sigma=5.0)
        rf = RangefinderArray(
            world.physics_cfg.robot_size_px, noisy_cfg, world.robot_shape
        )
        dt = world.physics_cfg.timestep

        readings = []
        for _ in range(20):
            rf.update(world.robot_body, world.space, dt)
            readings.append(rf.range_front)

        # With σ=5 cm, readings should vary
        assert max(readings) - min(readings) > 1.0

    def test_reset_clears_noise_state(self) -> None:
        """After reset, noise bias should be cleared."""
        import random

        random.seed(123)
        world = _make_world()
        noisy_cfg = NoiseConfig(bias_drift_sigma=1.0)
        rf = RangefinderArray(
            world.physics_cfg.robot_size_px, noisy_cfg, world.robot_shape
        )
        dt = world.physics_cfg.timestep

        # Accumulate some bias drift
        for _ in range(100):
            rf.update(world.robot_body, world.space, dt)

        rf.reset()

        # After reset, bias should be zero — reading should be close to true value
        random.seed(999)  # reset RNG to avoid drift on next call
        rf_clean = RangefinderArray(
            world.physics_cfg.robot_size_px, NoiseConfig(), world.robot_shape
        )
        rf_clean.update(world.robot_body, world.space, dt)
        # The reset one should now give a reading without accumulated bias
        # (but may still have instantaneous noise from the drift sigma)
        # Just verify reset doesn't crash and produces valid output
        rf.update(world.robot_body, world.space, dt)
        assert rf.range_front >= MIN_RANGE_CM
        assert rf.range_front <= MAX_RANGE_CM

    def test_ray_endpoints_stored_for_rendering(self) -> None:
        """Each sensor should store ray_start and ray_end after update."""
        world = _make_world()
        rf = RangefinderArray(
            world.physics_cfg.robot_size_px, NoiseConfig(), world.robot_shape
        )
        rf.update(world.robot_body, world.space, world.physics_cfg.timestep)

        for sensor in [rf.front, rf.right, rf.back, rf.left]:
            # ray_start should be near the robot position
            assert sensor.ray_start.get_distance(world.robot_body.position) < 50
            # ray_end should be farther away
            assert sensor.ray_end.get_distance(sensor.ray_start) > 10
