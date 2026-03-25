"""Tests for Renderer sizing behavior."""

from __future__ import annotations

import pygame

from robosim.config import SimulatorConfig
from robosim.renderer import Renderer


class _FakeFont:
    def render(self, _label: str, _antialias: bool, _colour: tuple[int, int, int]):
        return object()


class _FakeClock:
    def tick(self, _fps: int) -> None:
        return None


def _patch_renderer_init_pygame(monkeypatch) -> dict[str, tuple[int, int]]:
    captured: dict[str, tuple[int, int]] = {}

    def fake_set_mode(size: tuple[int, int]):
        captured["size"] = size
        return object()

    monkeypatch.setattr(pygame.display, "set_mode", fake_set_mode)
    monkeypatch.setattr(pygame.display, "set_caption", lambda _title: None)
    monkeypatch.setattr(pygame.font, "SysFont", lambda _name, _size: _FakeFont())
    monkeypatch.setattr(pygame.time, "Clock", lambda: _FakeClock())
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
    min_w = cfg.arena.arena_size_px + Renderer.SIDEBAR_WIDTH
    min_h = cfg.arena.arena_size_px

    renderer = Renderer(cfg)

    assert renderer.window_w == min_w
    assert renderer.window_h == min_h
    assert captured["size"] == (min_w, min_h)
