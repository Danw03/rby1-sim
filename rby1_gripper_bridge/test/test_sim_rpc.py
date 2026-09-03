import math

import pytest

from rby1_gripper_bridge.core import (
    GripperConfigurationError,
    GripperHardwareError,
)
from rby1_gripper_bridge.sim_rpc import (
    MuJoCoGripperDriver,
    decode_response_error,
    encode_initialization_request,
    encode_move_request,
)


def _length_field(field_number, payload):
    assert len(payload) < 128
    return bytes([(field_number << 3) | 2, len(payload)]) + payload


def _response_with_error(code, message=''):
    error = bytes([0x08, code])
    if message:
        error += _length_field(2, message.encode())
    header = _length_field(4, error)
    return _length_field(1, header)


class RecordingChannel:
    def __init__(self):
        self.calls = []
        self.responses = {}
        self.closed = False

    def unary_unary(
        self,
        method,
        request_serializer=None,
        response_deserializer=None,
    ):
        del request_serializer, response_deserializer

        def call(request, timeout):
            self.calls.append((method, request, timeout))
            response = self.responses.get(method, b'')
            if isinstance(response, Exception):
                raise response
            return response

        return call

    def close(self):
        self.closed = True


def test_public_proto_wire_encoding():
    assert encode_initialization_request('right') == b'\x12\x05right'
    assert encode_move_request('left', 25, 100, 50) == (
        b'\x12\x04left\x18\x19\x20\x64\x28\x32'
    )


def test_response_error_decoding():
    assert decode_response_error(b'') is None
    assert decode_response_error(_response_with_error(1)) is None
    assert decode_response_error(_response_with_error(3, 'bad request')) == (
        'CommonError code=3: bad request'
    )


def test_driver_initializes_both_sides_and_sends_percent_commands():
    channel = RecordingChannel()
    driver = MuJoCoGripperDriver(
        'localhost:50051',
        velocity_percent=80,
        force_percent=40,
        timeout_sec=1.5,
        channel=channel,
    )

    driver.initialize()
    assert driver.ready
    assert [call[1] for call in channel.calls] == [
        encode_initialization_request('right'),
        encode_initialization_request('left'),
    ]

    result = driver.command((0.255, 1.0))
    assert result == pytest.approx((0.255, 1.0))
    assert channel.calls[-2][1] == encode_move_request(
        'right', 26, 80, 40
    )
    assert channel.calls[-1][1] == encode_move_request(
        'left', 100, 80, 40
    )
    state = driver.read_state()
    assert state.close_ratios == pytest.approx((0.255, 1.0))
    assert all(math.isnan(value) for value in state.positions_rad)


def test_driver_rejects_server_application_error_and_becomes_unready():
    channel = RecordingChannel()
    driver = MuJoCoGripperDriver(channel=channel)
    driver.initialize()
    channel.responses[
        '/rb.api.GripperCommandService/GripperMove'
    ] = _response_with_error(3, 'invalid gripper')

    with pytest.raises(GripperHardwareError, match='invalid gripper'):
        driver.command((0.0, 0.0))

    assert not driver.ready
    assert not driver.healthy


def test_mujoco_torque_disable_is_explicitly_unsupported():
    driver = MuJoCoGripperDriver(channel=RecordingChannel())
    driver.initialize()

    with pytest.raises(GripperConfigurationError, match='no torque-disable'):
        driver.set_torque_enabled(False)


@pytest.mark.parametrize('bad_value', [-0.01, 1.01, math.nan])
def test_driver_rejects_invalid_normalized_command(bad_value):
    driver = MuJoCoGripperDriver(channel=RecordingChannel())
    driver.initialize()

    with pytest.raises(GripperConfigurationError):
        driver.command((bad_value, 0.5))
