"""Arena definition: tile grid and boundary wall geometry."""

from __future__ import annotations

import pymunk

from robosim.config import COLLISION_WALL, ArenaConfig


class Arena:
    """A 9x9 tile arena with static boundary walls."""

    def __init__(self, config: ArenaConfig, space: pymunk.Space) -> None:
        self.config = config
        self.walls: list[pymunk.Segment] = []
        self._create_boundary_walls(space)

    @property
    def size_px(self) -> int:
        return self.config.arena_size_px

    @property
    def tile_count(self) -> int:
        return self.config.tile_count

    @property
    def tile_size_px(self) -> int:
        return self.config.tile_size_px

    def _create_boundary_walls(self, space: pymunk.Space) -> None:
        """Create four boundary wall segments around the arena perimeter."""
        s = self.size_px
        corners = [(0, 0), (s, 0), (s, s), (0, s)]

        for i in range(4):
            a = corners[i]
            b = corners[(i + 1) % 4]
            seg = pymunk.Segment(space.static_body, a, b, self.config.wall_segment_radius)
            seg.elasticity = self.config.wall_elasticity
            seg.friction = self.config.wall_friction
            seg.collision_type = COLLISION_WALL
            space.add(seg)
            self.walls.append(seg)
