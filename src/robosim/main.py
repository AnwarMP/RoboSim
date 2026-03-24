"""Entry point — initializes everything and runs the game loop."""

from __future__ import annotations

import importlib
import math
import sys
from pathlib import Path

import pygame

from robosim.config import SimulatorConfig
from robosim.physics import PhysicsWorld
from robosim.renderer import Mode, Renderer
from robosim.robot import Robot
from robosim.types import DriveCommand, SensorPacket


def _load_user_script() -> object | None:
    """Try to import user_script from the current working directory."""
    script_path = Path.cwd() / "user_script.py"
    if not script_path.exists():
        return None
    # Ensure cwd is on sys.path so the import works
    cwd_str = str(Path.cwd())
    if cwd_str not in sys.path:
        sys.path.insert(0, cwd_str)
    try:
        mod = importlib.import_module("user_script")
        importlib.reload(mod)  # pick up latest edits
        return mod
    except Exception:
        return None


def _read_keyboard() -> DriveCommand:
    """Map held keys to a DriveCommand for manual mode."""
    keys = pygame.key.get_pressed()
    left = 0.0
    right = 0.0

    # Forward / backward
    if keys[pygame.K_UP] or keys[pygame.K_w]:
        left += 1.0
        right += 1.0
    if keys[pygame.K_DOWN] or keys[pygame.K_s]:
        left -= 1.0
        right -= 1.0

    # Turn left / right (differential)
    if keys[pygame.K_LEFT] or keys[pygame.K_a]:
        left -= 0.5
        right += 0.5
    if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        left += 0.5
        right -= 0.5

    return DriveCommand(
        left_power=max(-1.0, min(1.0, left)),
        right_power=max(-1.0, min(1.0, right)),
    )


def _build_sensor_packet(robot: Robot, sim_time: float) -> SensorPacket:
    """Build a basic SensorPacket from current robot state.

    Encoders and rangefinders are not yet wired (PRs 4-6); this provides
    IMU heading/angular velocity so the HUD is not blank.
    """
    body = robot.world.robot_body
    return SensorPacket(
        heading_deg=math.degrees(body.angle),
        angular_vel_deg=math.degrees(body.angular_velocity),
        timestamp=sim_time,
    )


def main() -> None:
    config = SimulatorConfig()

    pygame.init()
    renderer = Renderer(config)

    world = PhysicsWorld(config)
    robot = Robot(world)

    user_mod = _load_user_script()
    mode = Mode.MANUAL
    sim_time = 0.0
    error_msg = ""

    running = True
    while running:
        # -- Events ----------------------------------------------------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_TAB:
                    mode = Mode.AUTO if mode is Mode.MANUAL else Mode.MANUAL
                elif event.key == pygame.K_ESCAPE:
                    running = False

        # -- Drive command ---------------------------------------------------
        if mode is Mode.MANUAL:
            cmd = _read_keyboard()
        else:
            cmd = DriveCommand()
            if user_mod is not None and hasattr(user_mod, "run"):
                try:
                    sensors = _build_sensor_packet(robot, sim_time)
                    result = user_mod.run(sensors)
                    if isinstance(result, DriveCommand):
                        cmd = result
                        error_msg = ""
                    else:
                        error_msg = "run() must return DriveCommand"
                except Exception as exc:
                    error_msg = str(exc)[:120]
                    cmd = DriveCommand()
            else:
                error_msg = "No user_script.run() found"

        # -- Update ----------------------------------------------------------
        robot.update(cmd)
        sim_time += config.physics.timestep

        # -- Render ----------------------------------------------------------
        sensors = _build_sensor_packet(robot, sim_time)
        renderer.draw(robot, sensors, cmd, mode, sim_time, error_msg)
        renderer.tick(config.fps)

    pygame.quit()


if __name__ == "__main__":
    main()
