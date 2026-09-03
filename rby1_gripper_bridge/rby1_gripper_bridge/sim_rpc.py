"""Client for the gripper service exposed by the official MuJoCo server.

The public SDK ships the protobuf definition but no Python wrapper for this
service.  Using gRPC's generic unary API keeps this package independent of a
particular generated-protobuf version while still speaking the exact public
wire protocol.
"""

from __future__ import annotations

import math
import threading
from typing import Iterator, Optional, Sequence, Tuple, Union

from .core import (
    GripperConfigurationError,
    GripperHardwareError,
    GripperNotReadyError,
    GripperState,
)


Pair = Tuple[float, float]
WireValue = Union[int, bytes]
_SIDES = ('right', 'left')
_INITIALIZE_METHOD = (
    '/rb.api.GripperCommandService/GripperInitialization'
)
_MOVE_METHOD = '/rb.api.GripperCommandService/GripperMove'
_COMMON_ERROR_OK = 1


def _encode_varint(value: int) -> bytes:
    """Encode one non-negative protobuf varint."""

    value = int(value)
    if value < 0:
        raise ValueError('protobuf value must be non-negative')
    encoded = bytearray()
    while value > 0x7F:
        encoded.append((value & 0x7F) | 0x80)
        value >>= 7
    encoded.append(value)
    return bytes(encoded)


def _encode_string(field_number: int, value: str) -> bytes:
    payload = value.encode('utf-8')
    return (
        _encode_varint((field_number << 3) | 2)
        + _encode_varint(len(payload))
        + payload
    )


def _encode_int32(field_number: int, value: int) -> bytes:
    return _encode_varint(field_number << 3) + _encode_varint(value)


def encode_initialization_request(name: str) -> bytes:
    """Serialize ``GripperInitializationRequest``."""

    if name not in _SIDES:
        raise GripperConfigurationError(
            'MuJoCo gripper name must be "right" or "left"'
        )
    # request_header is optional.  The server only needs field 2 (name).
    return _encode_string(2, name)


def encode_move_request(
    name: str,
    position_percent: int,
    velocity_percent: int,
    force_percent: int,
) -> bytes:
    """Serialize ``GripperMoveRequest`` using the server's 0..100 units."""

    if name not in _SIDES:
        raise GripperConfigurationError(
            'MuJoCo gripper name must be "right" or "left"'
        )
    values = (
        int(position_percent),
        int(velocity_percent),
        int(force_percent),
    )
    if any(value < 0 or value > 100 for value in values):
        raise GripperConfigurationError(
            'MuJoCo position, velocity, and force must be in [0, 100]'
        )
    return b''.join((
        _encode_string(2, name),
        _encode_int32(3, values[0]),
        _encode_int32(4, values[1]),
        _encode_int32(5, values[2]),
    ))


def _decode_varint(payload: bytes, offset: int) -> Tuple[int, int]:
    value = 0
    shift = 0
    while offset < len(payload) and shift < 70:
        byte = payload[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, offset
        shift += 7
    raise GripperHardwareError('malformed protobuf varint in MuJoCo response')


def _iter_fields(payload: bytes) -> Iterator[Tuple[int, int, WireValue]]:
    offset = 0
    while offset < len(payload):
        key, offset = _decode_varint(payload, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if field_number == 0:
            raise GripperHardwareError(
                'malformed protobuf field in MuJoCo response'
            )
        if wire_type == 0:
            value, offset = _decode_varint(payload, offset)
            yield field_number, wire_type, value
        elif wire_type == 1:
            end = offset + 8
            if end > len(payload):
                raise GripperHardwareError(
                    'truncated protobuf field in MuJoCo response'
                )
            yield field_number, wire_type, payload[offset:end]
            offset = end
        elif wire_type == 2:
            size, offset = _decode_varint(payload, offset)
            end = offset + size
            if end > len(payload):
                raise GripperHardwareError(
                    'truncated protobuf field in MuJoCo response'
                )
            yield field_number, wire_type, payload[offset:end]
            offset = end
        elif wire_type == 5:
            end = offset + 4
            if end > len(payload):
                raise GripperHardwareError(
                    'truncated protobuf field in MuJoCo response'
                )
            yield field_number, wire_type, payload[offset:end]
            offset = end
        else:
            raise GripperHardwareError(
                f'unsupported protobuf wire type {wire_type} in response'
            )


def decode_response_error(payload: bytes) -> Optional[str]:
    """Return the server-side error message, or ``None`` on success."""

    response_header = None
    for field_number, wire_type, value in _iter_fields(payload):
        if field_number == 1 and wire_type == 2:
            response_header = value
            break
    if response_header is None:
        # An empty/default response is valid proto3 and has no error field.
        return None

    common_error = None
    assert isinstance(response_header, bytes)
    for field_number, wire_type, value in _iter_fields(response_header):
        if field_number == 4 and wire_type == 2:
            common_error = value
            break
    if common_error is None:
        return None

    code = 0
    message = ''
    assert isinstance(common_error, bytes)
    for field_number, wire_type, value in _iter_fields(common_error):
        if field_number == 1 and wire_type == 0:
            code = int(value)
        elif field_number == 2 and wire_type == 2:
            assert isinstance(value, bytes)
            message = value.decode('utf-8', errors='replace')
    if code == _COMMON_ERROR_OK:
        return None
    detail = message or 'no detail supplied'
    return f'CommonError code={code}: {detail}'


def _ratio_pair(values: Sequence[float]) -> Pair:
    if len(values) != 2:
        raise GripperConfigurationError(
            'close_ratios must contain exactly 2 values'
        )
    if any(isinstance(value, bool) for value in values):
        raise GripperConfigurationError('close_ratios must contain numbers')
    result = (float(values[0]), float(values[1]))
    if not all(math.isfinite(value) for value in result):
        raise GripperConfigurationError(
            'close_ratios must contain finite values'
        )
    if any(value < 0.0 or value > 1.0 for value in result):
        raise GripperConfigurationError(
            'close_ratios must be within [0.0, 1.0]'
        )
    return result


def _percent(value: float) -> int:
    # Round halves upward instead of Python's banker rounding.
    return int(math.floor(value * 100.0 + 0.5))


class MuJoCoGripperDriver:
    """Drive both simulated grippers through GripperCommandService."""

    def __init__(
        self,
        address: str = '127.0.0.1:50051',
        *,
        velocity_percent: int = 100,
        force_percent: int = 50,
        timeout_sec: float = 2.0,
        grpc_module=None,
        channel=None,
    ) -> None:
        address = str(address).strip()
        if not address:
            raise GripperConfigurationError('robot_address must not be empty')
        velocity_percent = int(velocity_percent)
        force_percent = int(force_percent)
        if not 0 <= velocity_percent <= 100:
            raise GripperConfigurationError(
                'mujoco_velocity_percent must be in [0, 100]'
            )
        if not 0 <= force_percent <= 100:
            raise GripperConfigurationError(
                'mujoco_force_percent must be in [0, 100]'
            )
        timeout_sec = float(timeout_sec)
        if not math.isfinite(timeout_sec) or timeout_sec <= 0.0:
            raise GripperConfigurationError('rpc_timeout_sec must be positive')

        if grpc_module is None and channel is None:
            try:
                import grpc as grpc_module  # type: ignore[no-redef]
            except ImportError as exc:
                raise GripperConfigurationError(
                    'Python grpc module is missing; install python3-grpcio'
                ) from exc

        self.address = address
        self.velocity_percent = velocity_percent
        self.force_percent = force_percent
        self.timeout_sec = timeout_sec
        self._grpc = grpc_module
        self._channel = channel
        if self._channel is None:
            self._channel = self._grpc.insecure_channel(address)
        self._initialize_rpc = self._channel.unary_unary(
            _INITIALIZE_METHOD,
            request_serializer=lambda value: value,
            response_deserializer=lambda value: value,
        )
        self._move_rpc = self._channel.unary_unary(
            _MOVE_METHOD,
            request_serializer=lambda value: value,
            response_deserializer=lambda value: value,
        )
        self._lock = threading.RLock()
        self._initialized = False
        self._healthy = False
        self._busy = False
        self._target_close_ratios: Optional[Pair] = None

    @property
    def initialized(self) -> bool:
        with self._lock:
            return self._initialized

    @property
    def healthy(self) -> bool:
        with self._lock:
            return self._healthy

    @property
    def enabled(self) -> bool:
        # The MuJoCo API exposes initialization/move, but no torque switch.
        return self.ready

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    @property
    def ready(self) -> bool:
        with self._lock:
            return self._initialized and self._healthy and not self._busy

    @property
    def target_close_ratios(self) -> Optional[Pair]:
        with self._lock:
            return self._target_close_ratios

    def initialize(self, force: bool = False) -> None:
        """Connect and initialize the server-side right and left grippers."""

        with self._lock:
            if self.ready and not force:
                return
            self._busy = True
            self._healthy = False
            self._initialized = False
            try:
                if self._grpc is not None:
                    self._grpc.channel_ready_future(self._channel).result(
                        timeout=self.timeout_sec
                    )
                for side in _SIDES:
                    self._call(
                        self._initialize_rpc,
                        encode_initialization_request(side),
                        f'GripperInitialization({side})',
                    )
                self._initialized = True
                self._healthy = True
            finally:
                self._busy = False

    def home(self) -> None:
        """Re-run simulator gripper initialization (no encoder homing)."""

        self.initialize(force=True)

    def command(self, close_ratios: Sequence[float]) -> Pair:
        """Move both sides; 0.0 is fully open and 1.0 fully closed."""

        ratios = _ratio_pair(close_ratios)
        with self._lock:
            if not self.ready:
                raise GripperNotReadyError(
                    'MuJoCo gripper RPC is not initialized or is unhealthy'
                )
            try:
                for side, ratio in zip(_SIDES, ratios):
                    request = encode_move_request(
                        side,
                        _percent(ratio),
                        self.velocity_percent,
                        self.force_percent,
                    )
                    self._call(
                        self._move_rpc,
                        request,
                        f'GripperMove({side})',
                    )
            except Exception:
                self._healthy = False
                raise
            self._target_close_ratios = ratios
            return ratios

    def repeat_last_command(self) -> None:
        """Skip refresh because the MuJoCo server latches move targets."""

    def read_state(self) -> GripperState:
        """Return the last target accepted by both RPC calls.

        GripperCommandService has no state RPC.  Consequently the normalized
        values are accepted targets, while motor-only measurements are NaN.
        """

        with self._lock:
            if not self._initialized:
                raise GripperNotReadyError(
                    'MuJoCo gripper RPC is not initialized'
                )
            ratios = self._target_close_ratios or (math.nan, math.nan)
            unavailable = (math.nan, math.nan)
            return GripperState(
                positions_rad=unavailable,
                velocities_rad_s=unavailable,
                currents_amp=unavailable,
                temperatures_c=unavailable,
                torque_enabled=(self.ready, self.ready),
                close_ratios=ratios,
            )

    def set_torque_enabled(self, enabled: bool) -> None:
        if not enabled:
            raise GripperConfigurationError(
                'MuJoCo GripperCommandService has no torque-disable RPC'
            )
        self.initialize(force=not self.ready)

    def shutdown(self, _disable_torque: bool = True) -> None:
        with self._lock:
            close = getattr(self._channel, 'close', None)
            if callable(close):
                close()
            self._initialized = False
            self._healthy = False
            self._target_close_ratios = None

    def _call(self, rpc, request: bytes, operation: str) -> None:
        try:
            response = rpc(request, timeout=self.timeout_sec)
        except Exception as exc:
            details = getattr(exc, 'details', None)
            detail = details() if callable(details) else str(exc)
            raise GripperHardwareError(
                f'MuJoCo RPC {operation} failed at {self.address}: {detail}'
            ) from exc
        error = decode_response_error(bytes(response))
        if error is not None:
            raise GripperHardwareError(
                f'MuJoCo RPC {operation} was rejected: {error}'
            )
