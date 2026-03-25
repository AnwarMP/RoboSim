"""Entry point — initializes everything and runs the game loop."""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path

import pygame

from robosim.config import NOISE_PRESETS, SimulatorConfig
from robosim.physics import PhysicsWorld
from robosim.renderer import Mode, Renderer
from robosim.robot import Robot
from robosim.sensors import EncoderPair, IMU, RangefinderArray
from robosim.types import DriveCommand, SensorPacket


def _load_user_script() -> tuple[object | None, str]:
    """Import or reload user_script from cwd, returning (module, error message)."""
    script_path = Path.cwd() / "user_script.py"
    if not script_path.exists():
        return None, "user_script.py not found"
    # Ensure cwd is on sys.path so the import works
    cwd_str = str(Path.cwd())
    if cwd_str not in sys.path:
        sys.path.insert(0, cwd_str)
    try:
        if "user_script" in sys.modules:
            mod = importlib.reload(sys.modules["user_script"])
        else:
            mod = importlib.import_module("user_script")
        return mod, ""
    except Exception as exc:
        return None, f"user_script import failed: {exc.__class__.__name__}: {exc}"


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
        left += 0.5
        right -= 0.5
    if keys[pygame.K_RIGHT] or keys[pygame.K_d]:
        left -= 0.5
        right += 0.5

    return DriveCommand(
        left_power=max(-1.0, min(1.0, left)),
        right_power=max(-1.0, min(1.0, right)),
    )


def _build_sensor_packet(
    robot: Robot,
    encoders: EncoderPair,
    imu: IMU,
    rangefinders: RangefinderArray,
    sim_time: float,
) -> SensorPacket:
    """Build a SensorPacket from current robot state."""
    return SensorPacket(
        enc_left=encoders.enc_left,
        enc_right=encoders.enc_right,
        heading_deg=imu.heading_deg,
        angular_vel_deg=imu.angular_vel_deg,
        range_front=rangefinders.range_front,
        range_right=rangefinders.range_right,
        range_back=rangefinders.range_back,
        range_left=rangefinders.range_left,
        timestamp=sim_time,
    )


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="robosim — 2D robot simulator")
    parser.add_argument(
        "--noise",
        choices=list(NOISE_PRESETS.keys()),
        default="ideal",
        help="Noise preset: ideal (default, zero noise), realistic, or stress",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = _parse_args(argv)
    noise_cfg = NOISE_PRESETS[args.noise]
    config = SimulatorConfig(
        encoder_noise=noise_cfg,
        imu_noise=noise_cfg,
        range_noise=noise_cfg,
    )

    pygame.init()
    renderer = Renderer(config)

    world = PhysicsWorld(config)
    robot = Robot(world)
    encoders = EncoderPair(config.physics, noise_cfg=config.encoder_noise)
    imu = IMU(world.robot_body.angle, config.imu_noise)
    rangefinders = RangefinderArray(
        config.physics.robot_size_px, config.range_noise, world.robot_shape
    )

    user_mod, script_load_error = _load_user_script()
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
            error_msg = ""
            cmd = _read_keyboard()
        else:
            cmd = DriveCommand()
            if user_mod is not None and hasattr(user_mod, "run"):
                try:
                    sensors = _build_sensor_packet(robot, encoders, imu, rangefinders, sim_time)
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
                error_msg = script_load_error or "No user_script.run() found"

        # -- Update ----------------------------------------------------------
        robot.update(cmd)
        body = robot.world.robot_body
        encoders.update(
            body.velocity, body.angle, body.angular_velocity,
            config.physics.timestep,
        )
        imu.update(body.angle, body.angular_velocity, config.physics.timestep)
        rangefinders.update(body, world.space, config.physics.timestep)
        sim_time += config.physics.timestep

        # -- Render ----------------------------------------------------------
        sensors = _build_sensor_packet(robot, encoders, imu, rangefinders, sim_time)
        renderer.draw(
            robot, sensors, cmd, mode, sim_time, error_msg,
            rangefinders=rangefinders, noise_preset=args.noise,
        )
        renderer.tick(config.fps)

    pygame.quit()


if __name__ == "__main__":
    main()
