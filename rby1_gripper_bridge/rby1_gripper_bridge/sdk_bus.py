"""Small compatibility adapter around ``rby1_sdk.DynamixelBus``."""

from __future__ import annotations

from importlib import metadata
from typing import Callable, Optional, Tuple


class SdkImportError(RuntimeError):
    """Raised when the vendor Python SDK is unavailable or incomplete."""


class SdkDynamixelBus:
    """Expose the subset of the SDK bus used by :class:`GripperDriver`."""

    def __init__(self, sdk_module, device_name: str) -> None:
        bus_type = getattr(sdk_module, 'DynamixelBus', None)
        if bus_type is None:
            raise SdkImportError(
                'rby1_sdk does not expose DynamixelBus; install a current SDK'
            )
        self._bus = bus_type(device_name)
        self.current_control_mode = int(bus_type.CurrentControlMode)
        self.current_based_position_control_mode = int(
            bus_type.CurrentBasedPositionControlMode
        )

    def open_port(self):
        return self._bus.open_port()

    def close_port(self) -> None:
        close = getattr(self._bus, 'close_port', None)
        if callable(close):
            close()

    def set_baud_rate(self, baud_rate):
        return self._bus.set_baud_rate(baud_rate)

    def set_torque_constant(self, values) -> None:
        self._bus.set_torque_constant(values)

    def ping(self, motor_id):
        return self._bus.ping(motor_id)

    def group_sync_write_torque_enable(self, ids, enabled) -> None:
        try:
            self._bus.group_sync_write_torque_enable(ids, enabled)
        except TypeError:
            self._bus.group_sync_write_torque_enable([
                (motor_id, enabled) for motor_id in ids
            ])

    def group_sync_write_operating_mode(self, values) -> None:
        self._bus.group_sync_write_operating_mode(values)

    def group_sync_write_send_torque(self, values) -> None:
        self._bus.group_sync_write_send_torque(values)

    def group_sync_write_send_position(self, values) -> None:
        self._bus.group_sync_write_send_position(values)

    def group_fast_sync_read_encoder(self, ids):
        return self._bus.group_fast_sync_read_encoder(ids)

    def get_motor_states(self, ids):
        return self._bus.get_motor_states(ids)


def create_sdk_bus(
    device_name: str = '',
    warning: Optional[Callable[[str], None]] = None,
) -> Tuple[SdkDynamixelBus, str, str]:
    """Return an SDK bus with its resolved path and package version."""

    try:
        import rby1_sdk as rby
    except ImportError as exc:
        raise SdkImportError(
            'rby1_sdk is not installed; install rby1-sdk in the ROS Python '
            'environment'
        ) from exc

    upc = getattr(rby, 'upc', None)
    if not device_name:
        device_name = str(
            getattr(upc, 'GripperDeviceName', '/dev/rby1_gripper')
        )

    initialize_device = getattr(upc, 'initialize_device', None)
    if callable(initialize_device):
        try:
            initialize_device(device_name)
        except Exception as exc:
            # The port open below is the authoritative availability test.
            if warning is not None:
                warning(
                    f'could not tune latency for {device_name}: {exc}; '
                    'continuing with the serial open check'
                )

    try:
        sdk_version = metadata.version('rby1-sdk')
    except metadata.PackageNotFoundError:
        sdk_version = 'unknown'
    return SdkDynamixelBus(rby, device_name), device_name, sdk_version
