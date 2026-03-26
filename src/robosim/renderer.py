"""Pygame rendering: arena grid, walls, robot, and HUD sidebar."""

from __future__ import annotations

import math
import textwrap
from enum import Enum, auto

import pygame

from robosim.config import PX_PER_CM, SimulatorConfig
from robosim.robot import Robot
from robosim.sensors import RangefinderArray
from robosim.sensors.rangefinder import MAX_RANGE_CM
from robosim.types import DriveCommand, SensorPacket


class Mode(Enum):
    MANUAL = auto()
    AUTO = auto()


class Renderer:
    """Draws the arena, robot, and telemetry sidebar each frame."""

    SIDEBAR_WIDTH = 220

    def __init__(self, config: SimulatorConfig) -> None:
        self.config = config
        self.arena_cfg = config.arena
        self.theme = config.ui_theme
        self.layout = config.ui_layout
        arena_px = self.arena_cfg.arena_size_px

        min_window_w = arena_px + self.layout.sidebar_width
        min_window_h = arena_px
        self.window_w = max(config.width_px, min_window_w)
        self.window_h = max(config.height_px, min_window_h)
        self.screen = pygame.display.set_mode((self.window_w, self.window_h))
        pygame.display.set_caption("robosim")

        self.font = pygame.font.SysFont(self.layout.font_name, self.layout.font_size)
        self.header_font = pygame.font.SysFont(self.layout.font_name, self.layout.header_font_size)
        self.clock = pygame.time.Clock()
        self._static_text_cache: dict[tuple[str, tuple[int, int, int], bool], pygame.Surface] = {}
        self._arena_surface: pygame.Surface | None = None

        self.arena_rect = pygame.Rect(0, 0, arena_px, arena_px)
        self.sidebar_rect = pygame.Rect(arena_px, 0, self.window_w - arena_px, self.window_h)
        self._compute_layout()

    def draw(
        self,
        robot: Robot,
        sensors: SensorPacket,
        cmd: DriveCommand,
        mode: Mode,
        sim_time: float,
        error_msg: str = "",
        rangefinders: RangefinderArray | None = None,
        noise_preset: str = "ideal",
    ) -> None:
        self.screen.fill(self.theme.window_bg)
        if self._arena_surface is None:
            self._arena_surface = self._build_static_arena(robot)
        self.screen.blit(self._arena_surface, self.arena_rect.topleft)

        if rangefinders is not None:
            self._draw_rangefinder_rays(rangefinders)
        self._draw_robot(robot)
        self._draw_sidebar(sensors, cmd, mode, sim_time, error_msg, noise_preset)
        pygame.display.flip()

    def tick(self, fps: int) -> None:
        self.clock.tick(fps)

    def _compute_layout(self) -> None:
        arena_px = self.arena_cfg.arena_size_px
        arena_y = max(0, (self.window_h - arena_px) // 2)
        self.arena_rect = pygame.Rect(0, arena_y, arena_px, arena_px)
        self.sidebar_rect = pygame.Rect(
            self.arena_rect.right,
            0,
            self.window_w - self.arena_rect.right,
            self.window_h,
        )

    def _build_static_arena(self, robot: Robot) -> pygame.Surface:
        arena_surface = pygame.Surface(self.arena_rect.size)
        arena_surface.fill(self.theme.arena_bg)
        self._draw_grid(arena_surface)
        self._draw_walls(arena_surface, robot)
        pygame.draw.rect(arena_surface, self.theme.arena_border, arena_surface.get_rect(), 2)
        return arena_surface

    def _arena_to_screen(self, x: float, y: float) -> tuple[int, int]:
        return (int(self.arena_rect.left + x), int(self.arena_rect.top + y))

    def _render_static_text(
        self,
        label: str,
        colour: tuple[int, int, int],
        header: bool = False,
    ) -> pygame.Surface:
        key = (label, colour, header)
        if key not in self._static_text_cache:
            font = self.header_font if header else self.font
            self._static_text_cache[key] = font.render(label, True, colour)
        return self._static_text_cache[key]

    # -- Arena drawing -------------------------------------------------------

    def _draw_grid(self, target: pygame.Surface) -> None:
        arena = self.arena_cfg
        size = arena.arena_size_px
        tile = arena.tile_size_px
        for i in range(arena.tile_count + 1):
            x = i * tile
            pygame.draw.line(target, self.theme.grid, (x, 0), (x, size))
            pygame.draw.line(target, self.theme.grid, (0, x), (size, x))

    def _draw_walls(self, target: pygame.Surface, robot: Robot) -> None:
        for wall in robot.world.arena.walls:
            a = (int(wall.a.x), int(wall.a.y))
            b = (int(wall.b.x), int(wall.b.y))
            pygame.draw.line(target, self.theme.wall, a, b, 3)

    # -- Rangefinder rays ----------------------------------------------------

    def _draw_rangefinder_rays(self, rangefinders: RangefinderArray) -> None:
        self.screen.set_clip(self.arena_rect)
        colours = [
            self.theme.ray_front,
            self.theme.ray_right,
            self.theme.ray_back,
            self.theme.ray_left,
        ]
        sensors = [rangefinders.front, rangefinders.right, rangefinders.back, rangefinders.left]
        for sensor, colour in zip(sensors, colours):
            start = self._arena_to_screen(sensor.ray_start.x, sensor.ray_start.y)

            dist_px = min(sensor.distance_cm, MAX_RANGE_CM) * PX_PER_CM
            ray_vec = sensor.ray_end - sensor.ray_start
            ray_len = ray_vec.length
            if ray_len > 0:
                end_point = sensor.ray_start + ray_vec * (dist_px / ray_len)
                end = self._arena_to_screen(end_point.x, end_point.y)
            else:
                end = start

            pygame.draw.line(self.screen, colour, start, end, 2)
            if sensor.distance_cm < MAX_RANGE_CM:
                pygame.draw.circle(self.screen, colour, end, 3)

        self.screen.set_clip(None)

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
        corners_local: list[tuple[float, float]] = []
        for dx, dy in [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]:
            rx = cx + dx * cos_a - dy * sin_a
            ry = cy + dx * sin_a + dy * cos_a
            corners_local.append((rx, ry))

        robot_surface = pygame.Surface(self.arena_rect.size, pygame.SRCALPHA)
        pygame.draw.polygon(robot_surface, self.theme.robot_fill, corners_local)
        pygame.draw.polygon(robot_surface, self.theme.robot_outline, corners_local, width=2)
        self.screen.blit(robot_surface, self.arena_rect.topleft)

        arrow_len = w / 2 + 8
        tip_x = cx + arrow_len * cos_a
        tip_y = cy + arrow_len * sin_a
        center = self._arena_to_screen(cx, cy)
        tip = self._arena_to_screen(tip_x, tip_y)
        pygame.draw.line(self.screen, self.theme.heading, center, tip, 2)

    # -- Sidebar / HUD -------------------------------------------------------

    def _draw_sidebar(
        self,
        sensors: SensorPacket,
        cmd: DriveCommand,
        mode: Mode,
        sim_time: float,
        error_msg: str,
        noise_preset: str = "ideal",
    ) -> None:
        pygame.draw.rect(self.screen, self.theme.sidebar_bg, self.sidebar_rect)
        pygame.draw.line(
            self.screen,
            self.theme.sidebar_border,
            self.sidebar_rect.topleft,
            self.sidebar_rect.bottomleft,
            2,
        )

        x = self.sidebar_rect.left + self.layout.sidebar_padding_x
        y = self.sidebar_rect.top + self.layout.sidebar_padding_y
        line_h = self.layout.line_height
        divider_right = self.sidebar_rect.right - self.layout.sidebar_padding_x

        def text(
            label: str,
            colour: tuple[int, int, int] | None = None,
            header: bool = False,
            static: bool = False,
        ) -> None:
            nonlocal y
            col = self.theme.text if colour is None else colour
            if static:
                surf = self._render_static_text(label, col, header=header)
            else:
                font = self.header_font if header else self.font
                surf = font.render(label, True, col)
            self.screen.blit(surf, (x, y))
            y += line_h

        def divider() -> None:
            nonlocal y
            pygame.draw.line(self.screen, self.theme.divider, (x, y), (divider_right, y), 1)
            y += self.layout.section_gap

        def metric(label: str, value: str) -> str:
            return f"{label:<8}{value:>{self.layout.value_width}}"

        mode_col = self.theme.mode_manual if mode is Mode.MANUAL else self.theme.mode_auto
        text("STATUS", colour=self.theme.section_text, header=True, static=True)
        text(metric("Mode", mode.name), colour=mode_col)
        text(metric("Noise", noise_preset.upper()))
        text(metric("Time", f"{sim_time:0.2f}s"))
        divider()

        text("DRIVE", colour=self.theme.section_text, header=True, static=True)
        text(metric("L pwr", f"{cmd.left_power:+.2f}"))
        text(metric("R pwr", f"{cmd.right_power:+.2f}"))
        divider()

        text("IMU", colour=self.theme.section_text, header=True, static=True)
        text(metric("Hdg", f"{sensors.heading_deg:0.1f} deg"))
        text(metric("Omega", f"{sensors.angular_vel_deg:0.1f} d/s"))
        divider()

        text("ENCODERS", colour=self.theme.section_text, header=True, static=True)
        text(metric("Enc L", f"{sensors.enc_left:d}"))
        text(metric("Enc R", f"{sensors.enc_right:d}"))
        divider()

        text("RANGE", colour=self.theme.section_text, header=True, static=True)
        text(metric("Front", f"{sensors.range_front:0.1f} cm"))
        text(metric("Right", f"{sensors.range_right:0.1f} cm"))
        text(metric("Back", f"{sensors.range_back:0.1f} cm"))
        text(metric("Left", f"{sensors.range_left:0.1f} cm"))
        divider()

        if error_msg:
            text("ERROR", colour=self.theme.error, header=True, static=True)
            wrapped = textwrap.wrap(error_msg, width=self.layout.error_wrap_chars)
            lines = wrapped[: self.layout.error_max_lines]
            if len(wrapped) > self.layout.error_max_lines and lines:
                lines[-1] = f"{lines[-1]}..."
            for line in lines:
                text(line, colour=self.theme.error)
        else:
            text("ERROR", colour=self.theme.text_muted, header=True, static=True)
            text("None", colour=self.theme.text_muted, static=True)

        controls = self._render_static_text("TAB: mode   ESC: quit", self.theme.text_muted)
        controls_y = self.sidebar_rect.bottom - self.layout.sidebar_padding_y - controls.get_height()
        self.screen.blit(controls, (x, controls_y))
