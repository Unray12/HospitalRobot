import huskylens_sensor.huskylens_sensor_reader as reader_module
from huskylens_sensor.huskylens_sensor_reader import HuskyLensSensorReader


def test_reconnect_tries_fallback_candidates_in_order(monkeypatch):
    reader = HuskyLensSensorReader.__new__(HuskyLensSensorReader)
    reader._logger = None
    reader.ser = None
    reader.port = "/dev/ttyACM2"
    reader.baudrate = 9600
    reader.timeout = 0.2

    attempts = []

    def fake_open(port):
        attempts.append(port)
        return port == "/dev/ttyACM1"

    reader._open_serial = fake_open
    reader._discover_ports = lambda prefixes: ["/dev/ttyACM3", "/dev/ttyACM1"]
    reader.is_connected = lambda: False

    ok = reader.reconnect(
        fallback_ports=["/dev/ttyACM0", "/dev/ttyACM1"],
        scan_prefixes=["/dev/ttyACM"],
    )

    assert ok is True
    assert attempts[0] == "/dev/ttyACM2"
    assert "/dev/ttyACM0" in attempts
    assert "/dev/ttyACM1" in attempts
