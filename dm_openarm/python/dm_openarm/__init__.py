from . import _core
from ._core import ArmConfig, ControlMode, MitCommand, MotorConfig, MotorModel, MotorState
from .arm import Arm
from .gravity_fit import (
    DEFAULT_COUPLED_BASIS,
    CoupledFitResult,
    build_phi,
    fit_coupled_all,
    format_coupled_yaml,
    predict_coupled,
)

__version__ = "0.1.0"

__all__ = [
    "Arm",
    "ArmConfig",
    "ControlMode",
    "MitCommand",
    "MotorConfig",
    "MotorModel",
    "MotorState",
    "CoupledFitResult",
    "DEFAULT_COUPLED_BASIS",
    "build_phi",
    "fit_coupled_all",
    "format_coupled_yaml",
    "predict_coupled",
    "_core",
]
