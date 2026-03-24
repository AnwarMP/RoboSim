"""Robot model: differential drive kinematics, P-controller, and lateral friction."""

from __future__ import annotations

import math

from pymunk import Vec2d

from robosim.physics import PhysicsWorld
from robosim.types import DriveCommand


class Robot:
    """Wraps a PhysicsWorld and applies differential-drive control each tick."""

    def __init__(self, world: PhysicsWorld) -> None:
        self.world = world
        self._cfg = world.physics_cfg

    @property
    def position(self) -> Vec2d:
        return self.world.robot_body.position

    @property
    def angle(self) -> float:
        return self.world.robot_body.angle

    @property
    def velocity(self) -> Vec2d:
        return self.world.robot_body.velocity

    @property
    def angular_velocity(self) -> float:
        return self.world.robot_body.angular_velocity

    def apply_drive(self, cmd: DriveCommand) -> None:
        """Apply a drive command: kinematics -> P-controller -> forces + lateral friction."""
        left = max(-1.0, min(1.0, cmd.left_power))
        right = max(-1.0, min(1.0, cmd.right_power))

        body = self.world.robot_body
        cfg = self._cfg

        # 1. Differential drive kinematics
        desired_forward = (left + right) / 2.0 * cfg.max_speed_px_s
        desired_omega = (right - left) / cfg.wheel_base_px * cfg.max_speed_px_s

        # 2. Forward direction from body angle
        forward = Vec2d(math.cos(body.angle), math.sin(body.angle))

        # 3. Project current velocity into body frame
        current_forward = body.velocity.dot(forward)

        # 4. P-control
        forward_force = (desired_forward - current_forward) * cfg.drive_gain
        torque = (desired_omega - body.angular_velocity) * self.world.torque_gain

        # 5. Apply force and torque
        body.apply_force_at_world_point(forward * forward_force, body.position)
        body.torque += torque

        # 6. Lateral friction impulse — cancel sideways sliding
        self._apply_lateral_friction()

    def _apply_lateral_friction(self) -> None:
        body = self.world.robot_body
        right = Vec2d(-math.sin(body.angle), math.cos(body.angle))
        lateral_speed = body.velocity.dot(right)
        impulse = -right * lateral_speed * body.mass * self._cfg.lateral_friction
        body.apply_impulse_at_world_point(impulse, body.position)

    def update(self, cmd: DriveCommand) -> None:
        """Full per-frame update: apply drive command then step physics."""
        self.apply_drive(cmd)
        self.world.step()
