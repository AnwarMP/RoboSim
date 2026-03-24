# RobotSim

RobotSim is a 2D top-down differential-drive robot simulator built with Python, Pymunk, and Pygame.  
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

This repository currently contains an initial scaffold and starter modules.  
The simulation loop and full physics/sensor systems are the next implementation step.

## Quick Start

```bash
# 1) Create virtual environment
uv venv

# 2) Activate it
source .venv/bin/activate

# 3) Install dependencies from pyproject.toml
uv sync

# 4) Run simulator entry point
uv run robosim

# 5) Run tests
uv run pytest
```

## User Code Contract

Write your autonomous logic in [`user_script.py`](RobotSim/user_script.py):

- `run(sensors: SensorPacket) -> DriveCommand`
- Called every simulation tick
- Should return quickly and always return a valid `DriveCommand`

## Architecture

The simulator uses these modules:

- [`main.py`](RobotSim/src/robosim/main.py): App entry point and top-level loop orchestration
- [`config.py`](RobotSim/src/robosim/config.py): Central dataclass-based configuration
- [`types.py`](RobotSim/src/robosim/types.py): Public API types (`SensorPacket`, `DriveCommand`)
- [`physics.py`](RobotSim/src/robosim/physics.py): Pymunk world setup and force/torque application
- [`robot.py`](RobotSim/src/robosim/robot.py): Robot drivetrain and state updates
- [`sensors.py`](RobotSim/src/robosim/sensors.py): Encoder/IMU/rangefinder simulation and noise pipeline
- [`renderer.py`](RobotSim/src/robosim/renderer.py): Pygame rendering and telemetry HUD
- [`field.py`](RobotSim/src/robosim/field.py): Arena layout and wall geometry

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

## Development Commands

```bash
# Lint (after adding ruff config/rules as needed)
uv run ruff check .

# Run tests
uv run pytest
```
