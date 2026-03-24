from dataclasses import dataclass, field


@dataclass(frozen=True)
class NoiseConfig:
    gaussian_sigma: float = 0.0
    bias_drift_sigma: float = 0.0


@dataclass(frozen=True)
class ArenaConfig:
    tile_count: int = 9
    tile_size_px: int = 72
    wall_segment_radius: float = 5.0
    wall_elasticity: float = 0.6
    wall_friction: float = 0.5

    @property
    def arena_size_px(self) -> int:
        return self.tile_count * self.tile_size_px


COLLISION_ROBOT = 1
COLLISION_WALL = 2

PX_PER_CM = 1.2


@dataclass(frozen=True)
class PhysicsConfig:
    timestep: float = 1.0 / 60.0
    space_damping: float = 0.8
    robot_mass: float = 5.0
    robot_size_px: tuple[float, float] = (60.0, 40.0)
    robot_friction: float = 0.7
    robot_elasticity: float = 0.4
    max_speed_px_s: float = 200.0
    wheel_base_px: float = 40.0
    drive_gain: float = 50.0
    torque_gain: float | None = None  # None = auto-calculate from moment
    lateral_friction: float = 1.0


@dataclass(frozen=True)
class SimulatorConfig:
    width_px: int = 900
    height_px: int = 700
    fps: int = 60
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    encoder_noise: NoiseConfig = field(default_factory=NoiseConfig)
    imu_noise: NoiseConfig = field(default_factory=NoiseConfig)
    range_noise: NoiseConfig = field(default_factory=NoiseConfig)
