# robosim

robosim is a 2D top-down differential-drive robot simulator built with Python, Pymunk, and Pygame.  
The project is designed so user code focuses on decision logic (`sense -> think -> act`) while the simulator handles physics, collisions, sensor modeling, and rendering.

## What This Project Does

- Simulates a rigid-body robot in a bounded 2D arena with wall collisions
- Exposes a simple user API:
  - Input: `SensorPacket`
  - Output: `DriveCommand`
  - Entry point: `run(sensors)` in `user_script.py`
- Supports differential-drive behaviors such as:
  - Straight driving
  - Point turns
  - Basic autonomous wall/heading logic
- Uses a modular codebase so physics, sensors, and rendering can evolve independently

## Current Status

The simulator has a working physics engine, differential-drive robot, and a live Pygame window with manual and autonomous driving modes.

## Quick Start

```bash
# 1) Create virtual environment and install dependencies
uv venv && source .venv/bin/activate && uv sync

# 2) Run the simulator
uv run robosim

# 3) Run tests
uv run pytest
```

## Controls

| Key | Action |
|-----|--------|
| `W` / `Up` | Drive forward |
| `S` / `Down` | Drive backward |
| `A` / `Left` | Turn left |
| `D` / `Right` | Turn right |
| `Tab` | Toggle between MANUAL and AUTO mode |
| `Esc` | Quit |

**MANUAL mode** (default): You drive the robot with the keyboard.

**AUTO mode**: The simulator calls `user_script.run(sensors)` each frame. The robot is controlled entirely by your code.

## User Code Contract

Write your autonomous logic in [`user_script.py`](user_script.py):

- `run(sensors: SensorPacket) -> DriveCommand`
- Called every simulation tick
- Should return quickly and always return a valid `DriveCommand`

## Architecture

The simulator uses these modules:

- [`main.py`](src/robosim/main.py): App entry point and top-level loop orchestration
- [`config.py`](src/robosim/config.py): Central dataclass-based configuration
- [`types.py`](src/robosim/types.py): Public API types (`SensorPacket`, `DriveCommand`)
- [`physics.py`](src/robosim/physics.py): Pymunk world setup and force/torque application
- [`robot.py`](src/robosim/robot.py): Robot drivetrain and state updates
- [`sensors.py`](src/robosim/sensors.py): Encoder/IMU/rangefinder simulation and noise pipeline
- [`renderer.py`](src/robosim/renderer.py): Pygame rendering and telemetry HUD
- [`field.py`](src/robosim/field.py): Arena layout and wall geometry

## Project Layout

```text
robosim/
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

## Development Commands

```bash
# Lint (after adding ruff config/rules as needed)
uv run ruff check .

# Run tests
uv run pytest
```
