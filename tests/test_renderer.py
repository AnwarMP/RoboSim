"""Tests for renderer sizing and UI drawing behavior."""

from __future__ import annotations

import pygame

from robosim.config import SimulatorConfig
from robosim.renderer import Mode, Renderer
from robosim.types import DriveCommand, SensorPacket


class _FakeClock:
    def tick(self, _fps: int) -> None:
        return None


def _patch_renderer_init_pygame(
    monkeypatch,
) -> dict[str, tuple[int, int] | list[tuple[str, tuple[int, int, int]]]]:
    captured: dict[str, tuple[int, int] | list[tuple[str, tuple[int, int, int]]]] = {}
    render_calls: list[tuple[str, tuple[int, int, int]]] = []

    def fake_set_mode(size: tuple[int, int]):
        captured["size"] = size
        return pygame.Surface(size)

    class _FakeFont:
        def render(self, label: str, _antialias: bool, colour: tuple[int, int, int]):
            render_calls.append((label, colour))
            return pygame.Surface((max(1, len(label)), 18))

    monkeypatch.setattr(pygame.display, "set_mode", fake_set_mode)
    monkeypatch.setattr(pygame.display, "set_caption", lambda _title: None)
    monkeypatch.setattr(pygame.font, "SysFont", lambda _name, _size: _FakeFont())
    monkeypatch.setattr(pygame.time, "Clock", lambda: _FakeClock())
    captured["render_calls"] = render_calls
    return captured


def test_renderer_uses_configured_window_size_when_large_enough(monkeypatch) -> None:
    captured = _patch_renderer_init_pygame(monkeypatch)
    cfg = SimulatorConfig(width_px=1200, height_px=800)

    renderer = Renderer(cfg)

    assert renderer.window_w == 1200
    assert renderer.window_h == 800
    assert captured["size"] == (1200, 800)


def test_renderer_enforces_minimum_size_for_arena_and_sidebar(monkeypatch) -> None:
    captured = _patch_renderer_init_pygame(monkeypatch)
    cfg = SimulatorConfig(width_px=100, height_px=100)
    min_w = cfg.arena.arena_size_px + cfg.ui_layout.sidebar_width
    min_h = cfg.arena.arena_size_px

    renderer = Renderer(cfg)

    assert renderer.window_w == min_w
    assert renderer.window_h == min_h
    assert captured["size"] == (min_w, min_h)


def test_renderer_centers_arena_when_window_is_taller(monkeypatch) -> None:
    _patch_renderer_init_pygame(monkeypatch)
    cfg = SimulatorConfig(width_px=900, height_px=900)

    renderer = Renderer(cfg)

    expected_y = (renderer.window_h - cfg.arena.arena_size_px) // 2
    assert renderer.arena_rect.y == expected_y
    assert renderer.sidebar_rect.left == renderer.arena_rect.right


def test_draw_orders_layers_and_reuses_cached_arena(monkeypatch) -> None:
    _patch_renderer_init_pygame(monkeypatch)
    renderer = Renderer(SimulatorConfig())
    calls: list[str] = []

    monkeypatch.setattr(
        renderer,
        "_build_static_arena",
        lambda _robot: calls.append("build") or pygame.Surface(renderer.arena_rect.size),
    )
    monkeypatch.setattr(renderer, "_draw_rangefinder_rays", lambda _rf: calls.append("rays"))
    monkeypatch.setattr(renderer, "_draw_robot", lambda _robot: calls.append("robot"))
    monkeypatch.setattr(renderer, "_draw_sidebar", lambda *_args, **_kw: calls.append("sidebar"))
    monkeypatch.setattr(pygame.display, "flip", lambda: calls.append("flip"))

    renderer.draw(
        robot=object(),  # type: ignore[arg-type]
        sensors=SensorPacket(),
        cmd=DriveCommand(),
        mode=Mode.MANUAL,
        sim_time=0.0,
        rangefinders=object(),  # type: ignore[arg-type]
    )
    renderer.draw(
        robot=object(),  # type: ignore[arg-type]
        sensors=SensorPacket(),
        cmd=DriveCommand(),
        mode=Mode.MANUAL,
        sim_time=0.1,
        rangefinders=object(),  # type: ignore[arg-type]
    )

    assert calls[:5] == ["build", "rays", "robot", "sidebar", "flip"]
    assert calls[5:] == ["rays", "robot", "sidebar", "flip"]


def test_sidebar_mode_uses_distinct_colours(monkeypatch) -> None:
    captured = _patch_renderer_init_pygame(monkeypatch)
    cfg = SimulatorConfig()
    renderer = Renderer(cfg)
    render_calls = captured["render_calls"]
    assert isinstance(render_calls, list)

    renderer._draw_sidebar(SensorPacket(), DriveCommand(), Mode.MANUAL, 0.0, "", "ideal")
    renderer._draw_sidebar(SensorPacket(), DriveCommand(), Mode.AUTO, 0.0, "", "ideal")

    mode_lines = [(label, colour) for label, colour in render_calls if "Mode" in label]
    assert any("MANUAL" in label and colour == cfg.ui_theme.mode_manual for label, colour in mode_lines)
    assert any("AUTO" in label and colour == cfg.ui_theme.mode_auto for label, colour in mode_lines)


def test_sidebar_error_wrapping_is_bounded(monkeypatch) -> None:
    captured = _patch_renderer_init_pygame(monkeypatch)
    cfg = SimulatorConfig()
    renderer = Renderer(cfg)
    render_calls = captured["render_calls"]
    assert isinstance(render_calls, list)

    long_error = "x" * 300
    renderer._draw_sidebar(SensorPacket(), DriveCommand(), Mode.MANUAL, 0.0, long_error, "ideal")

    error_lines = [
        label for label, colour in render_calls
        if colour == cfg.ui_theme.error and label != "ERROR"
    ]
    assert len(error_lines) == cfg.ui_layout.error_max_lines
    assert error_lines[-1].endswith("...")
