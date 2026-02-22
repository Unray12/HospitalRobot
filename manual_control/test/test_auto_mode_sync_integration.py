from manual_control.manual_control.auto_mode_sync import AutoModeSync


def test_auto_mode_retry_then_send_flow():
    sync = AutoModeSync(max_attempts=5)
    sync.queue(True)

    action_1, mode_1 = sync.step(service_ready=False)
    action_2, mode_2 = sync.step(service_ready=False)
    action_3, mode_3 = sync.step(service_ready=True)

    assert (action_1, mode_1) == ("wait", True)
    assert (action_2, mode_2) == ("wait", True)
    assert (action_3, mode_3) == ("send", True)


def test_auto_mode_give_up_after_max_attempts():
    sync = AutoModeSync(max_attempts=2)
    sync.queue(False)

    assert sync.step(service_ready=False) == ("wait", False)
    assert sync.step(service_ready=False) == ("give_up", False)
    assert sync.step(service_ready=False) == ("idle", None)
