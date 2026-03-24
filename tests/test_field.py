"""Tests for Arena (field.py) — wall geometry and tile grid."""

import pymunk

from robosim.config import COLLISION_WALL, ArenaConfig
from robosim.field import Arena


def _make_arena(config: ArenaConfig | None = None) -> tuple[Arena, pymunk.Space]:
    config = config or ArenaConfig()
    space = pymunk.Space()
    arena = Arena(config, space)
    return arena, space


class TestArenaDefaults:
    def test_default_arena_size(self) -> None:
        arena, _ = _make_arena()
        assert arena.size_px == 9 * 72  # 648

    def test_tile_count(self) -> None:
        arena, _ = _make_arena()
        assert arena.tile_count == 9

    def test_tile_size(self) -> None:
        arena, _ = _make_arena()
        assert arena.tile_size_px == 72


class TestBoundaryWalls:
    def test_four_wall_segments_created(self) -> None:
        arena, _ = _make_arena()
        assert len(arena.walls) == 4

    def test_walls_are_static_segments(self) -> None:
        arena, space = _make_arena()
        for wall in arena.walls:
            assert isinstance(wall, pymunk.Segment)
            assert wall.body is space.static_body

    def test_wall_collision_type(self) -> None:
        arena, _ = _make_arena()
        for wall in arena.walls:
            assert wall.collision_type == COLLISION_WALL

    def test_wall_elasticity_and_friction(self) -> None:
        arena, _ = _make_arena()
        for wall in arena.walls:
            assert wall.elasticity == 0.6
            assert wall.friction == 0.5

    def test_wall_segment_radius(self) -> None:
        arena, _ = _make_arena()
        for wall in arena.walls:
            assert wall.radius == 5.0

    def test_walls_form_closed_perimeter(self) -> None:
        """Verify the four walls connect to form a closed rectangle."""
        arena, _ = _make_arena()
        s = arena.size_px
        expected_corners = {(0, 0), (s, 0), (s, s), (0, s)}

        endpoints: set[tuple[float, float]] = set()
        for wall in arena.walls:
            a = wall.a
            b = wall.b
            endpoints.add((a.x, a.y))
            endpoints.add((b.x, b.y))

        assert endpoints == expected_corners

    def test_walls_added_to_space(self) -> None:
        _, space = _make_arena()
        # static_body shapes include the 4 wall segments
        static_shapes = space.static_body.shapes
        assert len(static_shapes) == 4


class TestCustomConfig:
    def test_custom_tile_count(self) -> None:
        config = ArenaConfig(tile_count=5, tile_size_px=100)
        arena, _ = _make_arena(config)
        assert arena.size_px == 500
        assert arena.tile_count == 5

    def test_custom_wall_properties(self) -> None:
        config = ArenaConfig(wall_elasticity=0.3, wall_friction=0.9, wall_segment_radius=2.0)
        arena, _ = _make_arena(config)
        for wall in arena.walls:
            assert wall.elasticity == 0.3
            assert wall.friction == 0.9
            assert wall.radius == 2.0
