"""Project-specific exceptions."""


class FruitSorterError(RuntimeError):
    """Base exception for controlled application failures."""


class ConfigurationError(FruitSorterError):
    """Raised when a configuration value is invalid or unsafe."""


class CameraError(FruitSorterError):
    """Raised when the camera cannot provide a valid frame."""


class DetectionError(FruitSorterError):
    """Raised when model loading or inference fails."""


class CalibrationError(FruitSorterError):
    """Raised when pixel-to-robot calibration is unusable."""


class KinematicsError(FruitSorterError):
    """Raised when a target is unreachable or violates a joint limit."""


class SafetyError(FruitSorterError):
    """Raised when a path violates a configured safety rule."""


class RobotCommunicationError(FruitSorterError):
    """Raised when Linux-to-MCU communication fails."""


class RobotFaultError(FruitSorterError):
    """Raised when the MCU reports an emergency or watchdog fault."""
