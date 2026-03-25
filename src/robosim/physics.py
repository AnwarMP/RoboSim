"""Pymunk world setup: space creation, robot body, and force application."""

from __future__ import annotations

import math

import pymunk

from robosim.config import COLLISION_ROBOT, SimulatorConfig
from robosim.field import Arena


class PhysicsWorld:
    """Manages the Pymunk space, robot body, and arena."""

    def __init__(self, config: SimulatorConfig) -> None:
        self.physics_cfg = config.physics
        self.arena_cfg = config.arena
        self.start_cfg = config.start

        # Create and configure the Pymunk space
        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        self.space.damping = self.physics_cfg.space_damping

        # Build arena walls
        self.arena = Arena(self.arena_cfg, self.space)

        # Create robot body at configured start position
        self.robot_body, self.robot_shape = self._create_robot()

        # Resolve torque gain (auto-calculate from moment if not provided)
        if self.physics_cfg.torque_gain is not None:
            self._torque_gain = self.physics_cfg.torque_gain
        else:
            self._torque_gain = 10.0 * self.robot_body.moment

    @property
    def torque_gain(self) -> float:
        return self._torque_gain

    def _create_robot(self) -> tuple[pymunk.Body, pymunk.Poly]:
        """Create the robot rigid body and collision shape at start position."""
        body = pymunk.Body()
        body.position = (self.start_cfg.x, self.start_cfg.y)
        body.angle = math.radians(self.start_cfg.heading_deg)

        w, h = self.physics_cfg.robot_size_px
        shape = pymunk.Poly.create_box(body, (w, h))
        shape.mass = self.physics_cfg.robot_mass
        shape.friction = self.physics_cfg.robot_friction
        shape.elasticity = self.physics_cfg.robot_elasticity
        shape.collision_type = COLLISION_ROBOT

        self.space.add(body, shape)
        return body, shape

    def step(self) -> None:
        """Advance the physics simulation by one fixed timestep."""
        self.space.step(self.physics_cfg.timestep)
