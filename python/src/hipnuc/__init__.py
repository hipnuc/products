"""HiPNUC serial and Modbus SDK. Importing never opens a port."""

from .decoder import Decoder
from .errors import DeviceError, HipnucError, ResponseTimeout, TransportError, VerificationError
from .models import CommandResult, DeviceInfo, Sample
from .modbus import ModbusBus, ModbusDevice, WriteResult
from .recording import Recorder
from .serial_device import DiscoveredDevice, DiscoveryResult, SerialDevice, discover

__version__ = "0.1.0"
__all__ = [
    "Decoder",
    "Sample",
    "DeviceInfo",
    "CommandResult",
    "SerialDevice",
    "Recorder",
    "discover",
    "DiscoveredDevice",
    "DiscoveryResult",
    "HipnucError",
    "DeviceError",
    "TransportError",
    "ResponseTimeout",
    "VerificationError",
    "ModbusBus",
    "ModbusDevice",
    "WriteResult",
]
