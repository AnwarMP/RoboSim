"""Tests for main.py helpers and loop behavior."""

from __future__ import annotations

import sys
from types import SimpleNamespace

import pygame

from robosim.main import _load_user_script, _read_keyboard
from robosim.types import DriveCommand, SensorPacket


class _PressedKeys:
    def __init__(self, pressed: set[int]) -> None:
        self._pressed = pressed

    def __getitem__(self, key: int) -> bool:
        return key in self._pressed


def _clear_user_script_module() -> None:
    sys.modules.pop("user_script", None)


class TestLoadUserScript:
    def test_missing_file_returns_clear_error(self, monkeypatch, tmp_path) -> None:
        monkeypatch.chdir(tmp_path)
        _clear_user_script_module()

        mod, err = _load_user_script()

        assert mod is None
        assert err == "user_script.py not found"

    def test_import_error_is_reported(self, monkeypatch, tmp_path) -> None:
        monkeypatch.chdir(tmp_path)
        (tmp_path / "user_script.py").write_text("def broken(:\n", encoding="utf-8")
        _clear_user_script_module()

        mod, err = _load_user_script()

        assert mod is None
        assert err.startswith("user_script import failed:")
        assert "SyntaxError" in err

    def test_first_load_does_not_double_execute_module(self, monkeypatch, tmp_path) -> None:
        monkeypatch.chdir(tmp_path)
        (tmp_path / "user_script.py").write_text(
            "import_count = globals().get('import_count', 0) + 1\n"
            "def run(_sensors):\n"
            "    return None\n",
            encoding="utf-8",
        )
        _clear_user_script_module()

        mod, err = _load_user_script()

        assert err == ""
        assert mod is not None
        assert getattr(mod, "import_count") == 1

    def test_subsequent_load_reloads_module(self, monkeypatch, tmp_path) -> None:
        monkeypatch.chdir(tmp_path)
        (tmp_path / "user_script.py").write_text(
            "import_count = globals().get('import_count', 0) + 1\n"
            "def run(_sensors):\n"
            "    return None\n",
            encoding="utf-8",
        )
        _clear_user_script_module()

        mod, _ = _load_user_script()
        mod, _ = _load_user_script()

        assert mod is not None
        assert getattr(mod, "import_count") == 2


class TestKeyboardMapping:
    def test_left_key_makes_left_wheel_faster_than_right(self, monkeypatch) -> None:
        monkeypatch.setattr(
            pygame.key,
            "get_pressed",
            lambda: _PressedKeys({pygame.K_LEFT}),
        )

        cmd = _read_keyboard()

        assert cmd.left_power == 0.5
        assert cmd.right_power == -0.5

    def test_right_key_makes_right_wheel_faster_than_left(self, monkeypatch) -> None:
        monkeypatch.setattr(
            pygame.key,
            "get_pressed",
            lambda: _PressedKeys({pygame.K_RIGHT}),
        )

        cmd = _read_keyboard()

        assert cmd.left_power == -0.5
        assert cmd.right_power == 0.5


def test_main_clears_auto_error_when_switching_back_to_manual(monkeypatch) -> None:
    import robosim.main as main_mod

    draw_calls: list[tuple[object, str]] = []

    class FakeRenderer:
        def __init__(self, _config) -> None:
            pass

        def draw(self, _robot, _sensors, _cmd, mode, _sim_time, error_msg, _rf=None) -> None:
            draw_calls.append((mode, error_msg))

        def tick(self, _fps: int) -> None:
            pass

    class FakeVec2d:
        def __init__(self, x: float = 0.0, y: float = 0.0) -> None:
            self.x, self.y = x, y

        def dot(self, _other: object) -> float:
            return 0.0

    class FakeWorld:
        def __init__(self, _config) -> None:
            self.robot_body = SimpleNamespace(
                angle=0.0, angular_velocity=0.0, velocity=FakeVec2d()
            )
            self.robot_shape = SimpleNamespace()
            self.space = SimpleNamespace()

    class FakeRobot:
        def __init__(self, world: FakeWorld) -> None:
            self.world = world

        def update(self, _cmd: DriveCommand) -> None:
            pass

    class FakeRangefinderArray:
        def __init__(self, _size, _noise, _shape) -> None:
            pass

        def update(self, _body, _space, _dt) -> None:
            pass

    events = [
        [SimpleNamespace(type=pygame.KEYDOWN, key=pygame.K_TAB)],  # switch to AUTO
        [SimpleNamespace(type=pygame.KEYDOWN, key=pygame.K_TAB)],  # back to MANUAL
        [SimpleNamespace(type=pygame.QUIT)],  # exit
    ]

    monkeypatch.setattr(main_mod.pygame, "init", lambda: None)
    monkeypatch.setattr(main_mod.pygame, "quit", lambda: None)
    monkeypatch.setattr(main_mod.pygame.event, "get", lambda: events.pop(0) if events else [])
    monkeypatch.setattr(main_mod, "Renderer", FakeRenderer)
    monkeypatch.setattr(main_mod, "PhysicsWorld", FakeWorld)
    monkeypatch.setattr(main_mod, "Robot", FakeRobot)
    monkeypatch.setattr(main_mod, "_read_keyboard", lambda: DriveCommand())
    monkeypatch.setattr(main_mod, "_build_sensor_packet", lambda _r, _e, _i, _rf, t: SensorPacket(timestamp=t))
    monkeypatch.setattr(main_mod, "RangefinderArray", FakeRangefinderArray)
    monkeypatch.setattr(main_mod, "_load_user_script", lambda: (None, "import failed"))

    main_mod.main()

    assert len(draw_calls) == 3
    assert draw_calls[0][1] == "import failed"
    assert draw_calls[1][1] == ""
