"""Pygame rendering: arena grid, walls, robot, and HUD sidebar."""

from __future__ import annotations

import math
from enum import Enum, auto

import pygame

from robosim.config import SimulatorConfig
from robosim.robot import Robot
from robosim.types import DriveCommand, SensorPacket

# Colours
COL_BG = (40, 40, 40)
COL_GRID = (70, 70, 70)
COL_WALL = (180, 210, 255)
COL_ROBOT = (60, 160, 80)
COL_HEADING = (255, 255, 255)
COL_SIDEBAR_BG = (30, 30, 30)
COL_TEXT = (200, 200, 200)
COL_MODE_MANUAL = (100, 200, 255)
COL_MODE_AUTO = (255, 180, 60)


class Mode(Enum):
    MANUAL = auto()
    AUTO = auto()


class Renderer:
    """Draws the arena, robot, and telemetry sidebar each frame."""

    SIDEBAR_WIDTH = 220

    def __init__(self, config: SimulatorConfig) -> None:
        self.config = config
        self.arena_cfg = config.arena
        arena_px = self.arena_cfg.arena_size_px

        self.window_w = arena_px + self.SIDEBAR_WIDTH
        self.window_h = arena_px
        self.screen = pygame.display.set_mode((self.window_w, self.window_h))
        pygame.display.set_caption("robosim")

        self.font = pygame.font.SysFont("monospace", 16)
        self.clock = pygame.time.Clock()

    def draw(
        self,
        robot: Robot,
        sensors: SensorPacket,
        cmd: DriveCommand,
        mode: Mode,
        sim_time: float,
        error_msg: str = "",
    ) -> None:
        self.screen.fill(COL_BG)
        self._draw_grid()
        self._draw_walls(robot)
        self._draw_robot(robot)
        self._draw_sidebar(sensors, cmd, mode, sim_time, error_msg)
        pygame.display.flip()

    def tick(self, fps: int) -> None:
        self.clock.tick(fps)

    # -- Arena drawing -------------------------------------------------------

    def _draw_grid(self) -> None:
        arena = self.arena_cfg
        size = arena.arena_size_px
        tile = arena.tile_size_px
        for i in range(arena.tile_count + 1):
            x = i * tile
            pygame.draw.line(self.screen, COL_GRID, (x, 0), (x, size))
            pygame.draw.line(self.screen, COL_GRID, (0, x), (size, x))

    def _draw_walls(self, robot: Robot) -> None:
        for wall in robot.world.arena.walls:
            a = (int(wall.a.x), int(wall.a.y))
            b = (int(wall.b.x), int(wall.b.y))
            pygame.draw.line(self.screen, COL_WALL, a, b, 3)

    # -- Robot drawing -------------------------------------------------------

    def _draw_robot(self, robot: Robot) -> None:
        body = robot.world.robot_body
        w, h = robot.world.physics_cfg.robot_size_px
        cx, cy = body.position
        angle = body.angle

        # Compute the four corners of the rotated rectangle
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        hw, hh = w / 2, h / 2
        corners = []
        for dx, dy in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]:
            rx = cx + dx * cos_a - dy * sin_a
            ry = cy + dx * sin_a + dy * cos_a
            corners.append((rx, ry))
        pygame.draw.polygon(self.screen, COL_ROBOT, corners)

        # Heading arrow — a line from center toward the front edge
        arrow_len = w / 2 + 8
        tip_x = cx + arrow_len * cos_a
        tip_y = cy + arrow_len * sin_a
        pygame.draw.line(self.screen, COL_HEADING, (int(cx), int(cy)), (int(tip_x), int(tip_y)), 2)

    # -- Sidebar / HUD -------------------------------------------------------

    def _draw_sidebar(
        self,
        sensors: SensorPacket,
        cmd: DriveCommand,
        mode: Mode,
        sim_time: float,
        error_msg: str,
    ) -> None:
        arena_px = self.arena_cfg.arena_size_px
        sidebar_rect = pygame.Rect(arena_px, 0, self.SIDEBAR_WIDTH, self.window_h)
        pygame.draw.rect(self.screen, COL_SIDEBAR_BG, sidebar_rect)

        x = arena_px + 12
        y = 14
        line_h = 22

        def text(label: str, colour: tuple[int, int, int] = COL_TEXT) -> None:
            nonlocal y
            surf = self.font.render(label, True, colour)
            self.screen.blit(surf, (x, y))
            y += line_h

        # Mode
        mode_col = COL_MODE_MANUAL if mode is Mode.MANUAL else COL_MODE_AUTO
        text(f"Mode: {mode.name}", mode_col)
        y += 6

        # Time
        text(f"Time:  {sim_time:7.2f}s")
        y += 6

        # Drive command
        text("-- Drive --")
        text(f"L pwr: {cmd.left_power:+.2f}")
        text(f"R pwr: {cmd.right_power:+.2f}")
        y += 6

        # IMU
        text("-- IMU --")
        text(f"Hdg:   {sensors.heading_deg:7.1f} deg")
        text(f"Omega: {sensors.angular_vel_deg:7.1f} d/s")
        y += 6

        # Encoders
        text("-- Encoders --")
        text(f"Enc L: {sensors.enc_left:>7d}")
        text(f"Enc R: {sensors.enc_right:>7d}")
        y += 6

        # Rangefinders
        text("-- Range --")
        text(f"Front: {sensors.range_front:6.1f} cm")
        text(f"Right: {sensors.range_right:6.1f} cm")
        text(f"Back:  {sensors.range_back:6.1f} cm")
        text(f"Left:  {sensors.range_left:6.1f} cm")

        # Error
        if error_msg:
            y += 10
            text("-- Error --", (255, 80, 80))
            # Wrap long error messages
            for i in range(0, len(error_msg), 24):
                text(error_msg[i : i + 24], (255, 80, 80))
