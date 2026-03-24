from robosim.types import DriveCommand


def test_drive_command_defaults() -> None:
    cmd = DriveCommand()
    assert cmd.left_power == 0.0
    assert cmd.right_power == 0.0
