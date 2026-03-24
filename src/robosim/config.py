from dataclasses import dataclass, field


@dataclass(frozen=True)
class NoiseConfig:
    gaussian_sigma: float = 0.0
    bias_drift_sigma: float = 0.0


@dataclass(frozen=True)
class PhysicsConfig:
    timestep: float = 1.0 / 60.0
    space_damping: float = 0.8
    robot_mass: float = 5.0
    robot_size_px: tuple[float, float] = (60.0, 40.0)
    max_speed_px_s: float = 200.0
    wheel_base_px: float = 30.0
    drive_gain: float = 50.0


@dataclass(frozen=True)
class SimulatorConfig:
    width_px: int = 900
    height_px: int = 700
    fps: int = 60
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    encoder_noise: NoiseConfig = field(default_factory=NoiseConfig)
    imu_noise: NoiseConfig = field(default_factory=NoiseConfig)
    range_noise: NoiseConfig = field(default_factory=NoiseConfig)
