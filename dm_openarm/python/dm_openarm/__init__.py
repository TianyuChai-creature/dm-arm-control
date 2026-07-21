from . import _core
from ._core import ArmConfig, ControlMode, MitCommand, MotorConfig, MotorModel, MotorState
from .arm import Arm, MoveResult
from .limb import Limb
from .gravity_fit import (
    DEFAULT_COUPLED_BASIS,
    CoupledFitResult,
    build_phi,
    fit_coupled_all,
    format_coupled_yaml,
    predict_coupled,
)
from .trajectory import Trajectory, plan_joint_trajectory, sample_quintic

__version__ = "0.1.0"

__all__ = [
    "Arm",
    "ArmConfig",
    "ControlMode",
    "Limb",
    "MitCommand",
    "MotorConfig",
    "MotorModel",
    "MotorState",
    "MoveResult",
    "Trajectory",
    "CoupledFitResult",
    "DEFAULT_COUPLED_BASIS",
    "build_phi",
    "fit_coupled_all",
    "format_coupled_yaml",
    "predict_coupled",
    "plan_joint_trajectory",
    "sample_quintic",
    "_core",
]
