from robot_common.robot_common.command_protocol import format_command, parse_command


def test_parse_command_direction_speed():
    assert parse_command("Forward:9") == ("Forward", 9)
    assert parse_command("RotateLeft,7") == ("RotateLeft", 7)


def test_parse_command_stop_and_invalid():
    assert parse_command("Stop:99") == ("Stop", 0)
    assert parse_command("BadCmd:10") == (None, None)


def test_parse_command_clamps_negative_speed():
    assert parse_command("Forward:-3") == ("Forward", 0)


def test_format_command():
    assert format_command("Forward", 6) == "Forward:6"
    assert format_command("Stop", 20) == "Stop:0"
    assert format_command("Invalid", 5) is None
