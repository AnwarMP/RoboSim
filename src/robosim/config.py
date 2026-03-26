from dataclasses import dataclass, field


@dataclass(frozen=True)
class NoiseConfig:
    gaussian_sigma: float = 0.0
    bias_drift_sigma: float = 0.0


# -- Noise presets -----------------------------------------------------------
NOISE_IDEAL = NoiseConfig()  # zero noise (default)
NOISE_REALISTIC = NoiseConfig(gaussian_sigma=0.01, bias_drift_sigma=0.001)
NOISE_STRESS = NoiseConfig(gaussian_sigma=0.05, bias_drift_sigma=0.005)

NOISE_PRESETS: dict[str, NoiseConfig] = {
    "ideal": NOISE_IDEAL,
    "realistic": NOISE_REALISTIC,
    "stress": NOISE_STRESS,
}


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
class StartConfig:
    x: float = 324.0  # Center of 648px arena
    y: float = 324.0
    heading_deg: float = 0.0  # Facing right (angle=0 in Y-down)


@dataclass(frozen=True)
class UITheme:
    window_bg: tuple[int, int, int] = (34, 34, 34)
    arena_bg: tuple[int, int, int] = (40, 40, 40)
    grid: tuple[int, int, int] = (70, 70, 70)
    wall: tuple[int, int, int] = (180, 210, 255)
    arena_border: tuple[int, int, int] = (140, 170, 215)
    sidebar_bg: tuple[int, int, int] = (24, 24, 24)
    sidebar_border: tuple[int, int, int] = (52, 52, 52)
    text: tuple[int, int, int] = (208, 208, 208)
    text_muted: tuple[int, int, int] = (150, 150, 150)
    section_text: tuple[int, int, int] = (120, 190, 255)
    mode_manual: tuple[int, int, int] = (100, 200, 255)
    mode_auto: tuple[int, int, int] = (255, 180, 60)
    error: tuple[int, int, int] = (255, 90, 90)
    divider: tuple[int, int, int] = (68, 68, 68)
    robot_fill: tuple[int, int, int, int] = (60, 160, 80, 160)
    robot_outline: tuple[int, int, int] = (90, 210, 120)
    heading: tuple[int, int, int] = (245, 245, 245)
    ray_front: tuple[int, int, int] = (255, 80, 80)
    ray_right: tuple[int, int, int] = (80, 200, 80)
    ray_back: tuple[int, int, int] = (255, 220, 60)
    ray_left: tuple[int, int, int] = (80, 220, 255)


@dataclass(frozen=True)
class UILayout:
    sidebar_width: int = 220
    sidebar_padding_x: int = 12
    sidebar_padding_y: int = 14
    line_height: int = 20
    section_gap: int = 8
    value_width: int = 10
    font_name: str = "monospace"
    font_size: int = 16
    header_font_size: int = 16
    error_wrap_chars: int = 26
    error_max_lines: int = 4


@dataclass(frozen=True)
class SimulatorConfig:
    width_px: int = 900
    height_px: int = 700
    fps: int = 60
    arena: ArenaConfig = field(default_factory=ArenaConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    start: StartConfig = field(default_factory=StartConfig)
    encoder_noise: NoiseConfig = field(default_factory=NoiseConfig)
    imu_noise: NoiseConfig = field(default_factory=NoiseConfig)
    range_noise: NoiseConfig = field(default_factory=NoiseConfig)
    ui_theme: UITheme = field(default_factory=UITheme)
    ui_layout: UILayout = field(default_factory=UILayout)
