# RobotSim

2D top-down differential-drive robot simulator built with Python, Pymunk, and Pygame.

This repository is scaffolded from the product notes in `.claude/` and is set up for `uv`.

## Quick Start

```bash
uv venv
source .venv/bin/activate
uv sync
uv run robosim
uv run pytest
```

## Project Layout

```text
RobotSim/
├── src/robosim/
│   ├── main.py
│   ├── config.py
│   ├── types.py
│   ├── physics.py
│   ├── robot.py
│   ├── sensors.py
│   ├── renderer.py
│   └── field.py
├── tests/
├── user_script.py
└── pyproject.toml
```
