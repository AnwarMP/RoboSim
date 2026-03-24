from robosim.types import DriveCommand, SensorPacket


def run(sensors: SensorPacket) -> DriveCommand:
    if sensors.range_front > 30.0:
        return DriveCommand(left_power=0.5, right_power=0.5)
    return DriveCommand(0.0, 0.0)
