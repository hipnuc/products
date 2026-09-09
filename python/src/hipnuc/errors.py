"""Exceptions shared by the device and firmware-update APIs."""


class HipnucError(Exception):
    """Base class for device communication and configuration errors."""


class TransportError(HipnucError):
    """The port could not be opened, read, or written."""


class ResponseTimeout(TransportError):
    """The requested response was not received before the deadline."""


class CommandTimeout(ResponseTimeout):
    """An ASCII command could not complete; ``sent`` distinguishes an unconfirmed write.

    ``sent=False`` means no command bytes were written. ``sent=True`` does not
    establish acknowledgement or whether the device executed the command.
    """

    def __init__(self, message: str, *, sent: bool):
        super().__init__(message)
        self.sent = sent


class DeviceError(HipnucError):
    """The device rejected a request; retain its original response."""

    def __init__(self, message: str, *, code: int | None = None, response: str = ""):
        super().__init__(message)
        self.code = code
        self.response = response


class VerificationError(DeviceError):
    """The response or configuration readback differs from the requested value."""
