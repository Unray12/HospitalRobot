from robot_common.command_protocol import format_command, parse_command


def test_parse_command_direction_speed():
    assert parse_command("Forward:9") == ("Forward", 9)
    assert parse_command("RotateLeft,7") == ("RotateLeft", 7)
    assert parse_command("Left 4") == ("Left", 4)


def test_parse_command_stop_and_invalid():
    assert parse_command("Stop:99") == ("Stop", 0)
    assert parse_command("BadCmd:10") == (None, None)
    assert parse_command("") == (None, None)
    assert parse_command(None) == (None, None)


def test_parse_command_normalizes_speed_values():
    assert parse_command("Forward:-3") == ("Forward", 0)
    assert parse_command("Forward:8.8") == ("Forward", 9)
    assert parse_command("Forward:not-a-number") == ("Forward", 0)
    assert parse_command("Forward:nan") == ("Forward", 0)
    assert parse_command("Forward:inf") == ("Forward", 0)


def test_parse_command_trims_whitespace_and_ignores_extra_segments():
    assert parse_command("  Forward:5  ") == ("Forward", 5)
    assert parse_command("Forward:3:ignored") == ("Forward", 3)
    assert parse_command("Forward,3,ignored") == ("Forward", 3)


def test_format_command():
    assert format_command("Forward", 6) == "Forward:6"
    assert format_command("Forward", 6.6) == "Forward:7"
    assert format_command("Forward", "4.2") == "Forward:4"
    assert format_command("RotateRight", "bad") == "RotateRight:0"
    assert format_command("RotateRight", -5) == "RotateRight:0"
    assert format_command("Stop", 20) == "Stop:0"
    assert format_command("Invalid", 5) is None
