"""Rangefinder sensors — 4 directional raycasts."""

from __future__ import annotations

import math

import pymunk
from pymunk import Vec2d

from robosim.config import NoiseConfig, PX_PER_CM
from robosim.sensors.noise import NoisePipeline

# Range limits in centimetres
MAX_RANGE_CM: float = 200.0
MIN_RANGE_CM: float = 5.0

# Ray radius for segment queries (px)
RAY_RADIUS: float = 1.0

# Cardinal directions in the robot's local frame (body-relative).
# "Front" is +x (angle=0), using Y-down coordinates.
_LOCAL_DIRECTIONS: dict[str, Vec2d] = {
    "front": Vec2d(1.0, 0.0),
    "right": Vec2d(0.0, 1.0),   # +y = down = right in Y-down top-down
    "back": Vec2d(-1.0, 0.0),
    "left": Vec2d(0.0, -1.0),
}


class Rangefinder:
    """A single directional rangefinder using Pymunk raycasts.

    The sensor is mounted at the centre of one face of the robot's
    rectangular body and casts a ray in the corresponding direction.
    """

    def __init__(
        self,
        name: str,
        local_direction: Vec2d,
        robot_half_size: tuple[float, float],
        noise_cfg: NoiseConfig,
    ) -> None:
        self.name = name
        self._local_dir = local_direction
        self._noise = NoisePipeline(noise_cfg)

        # Mount offset: centre of the face in the direction of the ray.
        hw, hh = robot_half_size
        self._mount_offset = Vec2d(
            local_direction.x * hw,
            local_direction.y * hh,
        )

        self._last_distance_cm: float = MAX_RANGE_CM
        # Store the world-space start/end for rendering
        self.ray_start: Vec2d = Vec2d(0.0, 0.0)
        self.ray_end: Vec2d = Vec2d(0.0, 0.0)

    def update(
        self,
        body: pymunk.Body,
        space: pymunk.Space,
        shape_filter: pymunk.ShapeFilter,
        dt: float,
    ) -> None:
        """Cast a ray and update the distance reading."""
        max_range_px = MAX_RANGE_CM * PX_PER_CM

        # Transform mount point and direction to world frame
        self.ray_start = body.local_to_world(self._mount_offset)

        # Rotate local direction by body angle
        cos_a = math.cos(body.angle)
        sin_a = math.sin(body.angle)
        world_dir = Vec2d(
            self._local_dir.x * cos_a - self._local_dir.y * sin_a,
            self._local_dir.x * sin_a + self._local_dir.y * cos_a,
        )
        self.ray_end = self.ray_start + world_dir * max_range_px

        result = space.segment_query_first(
            self.ray_start, self.ray_end, RAY_RADIUS, shape_filter
        )

        if result and result.shape is not None:
            distance_px = result.alpha * max_range_px
        else:
            distance_px = max_range_px

        true_cm = distance_px / PX_PER_CM
        noisy_cm = self._noise.apply(true_cm, dt)
        self._last_distance_cm = max(MIN_RANGE_CM, min(MAX_RANGE_CM, noisy_cm))

    @property
    def distance_cm(self) -> float:
        return self._last_distance_cm

    def reset(self) -> None:
        self._noise.reset()
        self._last_distance_cm = MAX_RANGE_CM


class RangefinderArray:
    """Four rangefinders: front, right, back, left."""

    def __init__(
        self,
        robot_size_px: tuple[float, float],
        noise_cfg: NoiseConfig,
        robot_shape: pymunk.Shape,
    ) -> None:
        hw = robot_size_px[0] / 2.0
        hh = robot_size_px[1] / 2.0
        half_size = (hw, hh)

        self.front = Rangefinder("front", _LOCAL_DIRECTIONS["front"], half_size, noise_cfg)
        self.right = Rangefinder("right", _LOCAL_DIRECTIONS["right"], half_size, noise_cfg)
        self.back = Rangefinder("back", _LOCAL_DIRECTIONS["back"], half_size, noise_cfg)
        self.left = Rangefinder("left", _LOCAL_DIRECTIONS["left"], half_size, noise_cfg)

        self._sensors = [self.front, self.right, self.back, self.left]

        # ShapeFilter to exclude the robot's own shape from raycasts.
        # We assign the robot shape to a non-zero group and use the same
        # group in our query filter so the ray ignores the robot body.
        robot_group = 1
        robot_shape.filter = pymunk.ShapeFilter(group=robot_group)
        self._filter = pymunk.ShapeFilter(group=robot_group)

    def update(self, body: pymunk.Body, space: pymunk.Space, dt: float) -> None:
        """Update all four rangefinders."""
        for sensor in self._sensors:
            sensor.update(body, space, self._filter, dt)

    @property
    def range_front(self) -> float:
        return self.front.distance_cm

    @property
    def range_right(self) -> float:
        return self.right.distance_cm

    @property
    def range_back(self) -> float:
        return self.back.distance_cm

    @property
    def range_left(self) -> float:
        return self.left.distance_cm

    def reset(self) -> None:
        for sensor in self._sensors:
            sensor.reset()
