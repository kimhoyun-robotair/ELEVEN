import math

from aprl_navigation.command_mux import VelocityGate


def test_manual_priority_stop_and_expired_navigation_permit():
    gate = VelocityGate()
    gate.receive("navigation", 0.2, 0.1, 10.0)
    assert gate.output(10.0) == (0.0, 0.0)
    gate.inhibit(False, 10.0)
    assert gate.output(10.0) == (0.2, 0.1)
    gate.receive("manual", 0.0, 0.0, 10.1)
    assert gate.output(10.1) == (0.0, 0.0)
    gate.inhibit(False, 10.5)
    gate.receive("navigation", 0.2, 0.0, 10.5)
    assert gate.output(10.5) == (0.2, 0.0)
    gate.inhibit(True, 10.6)
    assert gate.output(10.6) == (0.0, 0.0)
    gate.inhibit(False, 11.0)
    gate.receive("navigation", 0.2, 0.0, 11.3)
    assert gate.output(11.5) == (0.0, 0.0)


def test_invalid_or_stale_commands_stop_and_speed_is_bounded():
    gate = VelocityGate()
    gate.receive("manual", 3.0, -4.0, 10.0)
    assert gate.output(10.0) == (0.3, -0.6)
    assert gate.output(10.5) == (0.0, 0.0)
    gate.receive("manual", math.nan, math.inf, 11.0)
    assert gate.output(11.0) == (0.0, 0.0)
