from dataclasses import dataclass


@dataclass(frozen=True)
class SensorPacket:
    enc_left: int = 0
    enc_right: int = 0
    heading_deg: float = 0.0
    angular_vel_deg: float = 0.0
    range_front: float = 200.0
    range_right: float = 200.0
    range_back: float = 200.0
    range_left: float = 200.0
    timestamp: float = 0.0


@dataclass
class DriveCommand:
    left_power: float = 0.0
    right_power: float = 0.0
