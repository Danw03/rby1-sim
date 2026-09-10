from dataclasses import dataclass

import pytest

from rby1_gripper_bridge.core import (
    Calibration,
    GripperConfigurationError,
    GripperDriver,
    GripperNotReadyError,
)


@dataclass
class FakeMotorState:
    torque_enable: bool
    position: float
    velocity: float = 0.0
    current: float = 0.0
    temperature: int = 25


class FakeBus:
    current_control_mode = 0
    current_based_position_control_mode = 5

    def __init__(self):
        self.ids = (0, 1)
        self.opened = False
        self.closed = False
        self.enabled = {0: False, 1: False}
        self.mode = {0: 5, 1: 5}
        self.position = {0: 0.0, 1: 0.0}
        self.torque = {0: 0.0, 1: 0.0}
        self.targets = {0: 0.0, 1: 0.0}

    def open_port(self):
        self.opened = True
        return True

    def close_port(self):
        self.closed = True

    def set_baud_rate(self, _baud_rate):
        return True

    def set_torque_constant(self, _values):
        return None

    def ping(self, motor_id):
        return motor_id in self.ids

    def group_sync_write_torque_enable(self, ids, enabled):
        for motor_id in ids:
            self.enabled[motor_id] = bool(enabled)

    def group_sync_write_operating_mode(self, values):
        for motor_id, mode in values:
            self.mode[motor_id] = mode

    def group_sync_write_send_torque(self, values):
        for motor_id, torque in values:
            self.torque[motor_id] = torque

    def group_sync_write_send_position(self, values):
        for motor_id, position in values:
            self.targets[motor_id] = position
            if self.mode[motor_id] == self.current_based_position_control_mode:
                self.position[motor_id] = position

    def group_fast_sync_read_encoder(self, ids):
        return [(motor_id, self.position[motor_id]) for motor_id in ids]

    def get_motor_states(self, ids):
        return [
            (
                motor_id,
                FakeMotorState(
                    torque_enable=self.enabled[motor_id],
                    position=self.position[motor_id],
                    current=self.torque[motor_id],
                ),
            )
            for motor_id in ids
        ]

    def advance(self):
        for motor_id in self.ids:
            if self.enabled[motor_id] and self.mode[motor_id] == 0:
                self.position[motor_id] = max(
                    -1.0,
                    min(
                        1.0,
                        self.position[motor_id]
                        + (1.0 if self.torque[motor_id] > 0.0 else -1.0)
                        * 0.5,
                    ),
                )


class FakeTime:
    def __init__(self, bus):
        self.now = 0.0
        self.bus = bus

    def clock(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        self.bus.advance()


def make_driver():
    bus = FakeBus()
    fake_time = FakeTime(bus)
    driver = GripperDriver(
        bus,
        clock=fake_time.clock,
        sleeper=fake_time.sleep,
    )
    driver.initialize()
    return driver, bus


def test_calibration_maps_open_and_closed_with_margin():
    calibration = Calibration(
        minimum_rad=(-2.0, -4.0),
        maximum_rad=(2.0, 4.0),
        endpoint_margin_ratio=0.1,
    )

    assert calibration.positions_for((0.0, 0.0)) == pytest.approx((1.6, 3.2))
    assert calibration.positions_for((1.0, 1.0)) == pytest.approx((-1.6, -3.2))
    assert calibration.close_ratios_for((1.6, -3.2)) == pytest.approx(
        (0.0, 1.0)
    )


def test_command_requires_calibration_and_enabled_torque():
    driver, _bus = make_driver()

    with pytest.raises(GripperNotReadyError, match='homed'):
        driver.command((0.0, 0.0))

    driver.load_calibration(
        Calibration(minimum_rad=(-1.0, -1.0), maximum_rad=(1.0, 1.0))
    )
    with pytest.raises(GripperNotReadyError, match='disabled'):
        driver.command((0.0, 0.0))


def test_homing_finds_endpoints_and_enables_normalized_commands():
    driver, bus = make_driver()

    calibration = driver.home(
        sample_period_sec=0.1,
        stall_threshold_rad=0.0,
        stall_samples=2,
        direction_timeout_sec=3.0,
    )

    assert calibration.minimum_rad == pytest.approx((-1.0, -1.0))
    assert calibration.maximum_rad == pytest.approx((1.0, 1.0))
    assert driver.ready

    positions = driver.command((0.0, 1.0))
    assert positions == pytest.approx((1.0, -1.0))
    assert bus.targets == pytest.approx({0: 1.0, 1: -1.0})

    state = driver.read_state()
    assert state.close_ratios == pytest.approx((0.0, 1.0))


def test_out_of_range_command_is_rejected_instead_of_clamped():
    driver, _bus = make_driver()
    driver.load_calibration(
        Calibration(minimum_rad=(-1.0, -1.0), maximum_rad=(1.0, 1.0))
    )
    driver.set_torque_enabled(True)

    with pytest.raises(GripperConfigurationError, match=r'\[0.0, 1.0\]'):
        driver.command((-0.1, 0.5))


def test_shutdown_disables_torque_and_closes_bus():
    driver, bus = make_driver()
    driver.load_calibration(
        Calibration(minimum_rad=(-1.0, -1.0), maximum_rad=(1.0, 1.0))
    )
    driver.set_torque_enabled(True)

    driver.shutdown()

    assert not any(bus.enabled.values())
    assert bus.closed
    assert not driver.initialized
