from __future__ import annotations

from robosim.config import SimulatorConfig
from robosim.types import SensorPacket


def main() -> None:
    config = SimulatorConfig()
    sensors = SensorPacket()
    print("RobotSim scaffold is ready.")
    print(f"Window: {config.width_px}x{config.height_px} @ {config.fps} FPS")
    print("Next: implement physics loop and call user_script.run(sensors).")


if __name__ == "__main__":
    main()
