# robosim

A 2D top-down differential-drive robot simulator built with Python, Pymunk, and Pygame.
Write a single `run(sensors) -> DriveCommand` function and the simulator handles physics, collisions, sensor noise, and rendering.

## What This Project Does

- Simulates a rigid-body robot in a bounded 9x9 tile arena with wall collisions
- Differential-drive kinematics: straight driving, point turns, arc turns
- Full sensor suite: wheel encoders, IMU (heading + angular velocity), 4 rangefinders
- Configurable noise pipeline with presets (ideal, realistic, stress)
- Live Pygame window with arena view, sensor ray overlays, and telemetry sidebar
- Manual (keyboard) and autonomous (user script) driving modes

## Quick Start

```bash
# Install dependencies
uv venv && source .venv/bin/activate && uv sync

# Run the simulator (zero noise by default)
uv run robosim

# Run with realistic sensor noise
uv run robosim --noise realistic

# Run with heavy noise (stress test)
uv run robosim --noise stress

# Run tests
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

**MANUAL mode** (default): Drive the robot with the keyboard.

**AUTO mode**: The simulator calls `user_script.run(sensors)` each frame. The robot is controlled entirely by your code.

## Writing a User Script

Create a `user_script.py` in the project root:

```python
from robosim.types import SensorPacket, DriveCommand

def run(sensors: SensorPacket) -> DriveCommand:
    """Called every tick (~60 Hz). Must return quickly."""
    if sensors.range_front > 30.0:
        return DriveCommand(left_power=0.5, right_power=0.5)
    return DriveCommand(0.0, 0.0)
```

### SensorPacket

| Field | Type | Description |
|-------|------|-------------|
| `enc_left` | `int` | Cumulative left wheel encoder ticks |
| `enc_right` | `int` | Cumulative right wheel encoder ticks |
| `heading_deg` | `float` | Heading in degrees (0 = start, CCW positive) |
| `angular_vel_deg` | `float` | Angular velocity in degrees/second |
| `range_front` | `float` | Front rangefinder distance in cm (5-200) |
| `range_right` | `float` | Right rangefinder distance in cm (5-200) |
| `range_back` | `float` | Back rangefinder distance in cm (5-200) |
| `range_left` | `float` | Left rangefinder distance in cm (5-200) |
| `timestamp` | `float` | Simulation time in seconds |

### DriveCommand

| Field | Type | Description |
|-------|------|-------------|
| `left_power` | `float` | Left motor power, clamped to [-1.0, 1.0] |
| `right_power` | `float` | Right motor power, clamped to [-1.0, 1.0] |

## Noise Presets

| Preset | Gaussian sigma | Bias drift sigma | Use case |
|--------|---------------|-----------------|----------|
| `ideal` | 0.0 | 0.0 | Algorithm debugging (default) |
| `realistic` | 0.01 | 0.001 | Simulating real MEMS sensor behavior |
| `stress` | 0.05 | 0.005 | Testing algorithm robustness |

Noise is applied uniformly to all sensors (encoders, IMU, rangefinders). With zero noise, sensor readings are exact. With noise enabled, heading drifts over time (mimicking real gyroscope behavior) and rangefinder readings jitter.

## Architecture

```text
src/robosim/
├── main.py              # Entry point, game loop, CLI parsing
├── config.py            # Dataclass configs, noise presets
├── types.py             # Public API types (SensorPacket, DriveCommand)
├── physics.py           # Pymunk space, robot body, force control
├── robot.py             # Differential-drive kinematics
├── sensors/
│   ├── noise.py         # Shared two-stage noise pipeline
│   ├── encoders.py      # Wheel encoder tick accumulation
│   ├── imu.py           # Heading and angular velocity
│   └── rangefinder.py   # 4-directional Pymunk raycasts
├── renderer.py          # Pygame rendering and telemetry HUD
└── field.py             # Arena layout and wall geometry
```

## Development

```bash
# Lint
uv run ruff check .

# Run tests
uv run pytest

# Run tests with verbose output
uv run pytest -v
```
