"""Exceptions shared by the serial and Modbus APIs."""


class HipnucError(Exception):
    """Base class for device communication and configuration errors."""


class TransportError(HipnucError):
    """The port could not be opened, read, or written."""


class ResponseTimeout(TransportError):
    """The requested response was not received before the deadline."""


class DeviceError(HipnucError):
    """The device rejected a request; retain its original response."""

    def __init__(self, message: str, *, code: int | None = None, response: str = ""):
        super().__init__(message)
        self.code = code
        self.response = response


class VerificationError(DeviceError):
    """The response or configuration readback differs from the requested value."""
