"""Shared interfaces and utilities for the depalletizing project."""

from .errors import (
    ERROR_BUSY,
    ERROR_CANCELLED,
    ERROR_FAULTED,
    ERROR_GOAL_REJECTED,
    ERROR_GRIPPER_FAILED,
    ERROR_MOVE_FAILED,
    ERROR_OUT_OF_WORKSPACE,
    ERROR_SERVER_UNAVAILABLE,
    ERROR_TIMEOUT,
    ERROR_WORKFLOW_BUSY,
    ERROR_WORKFLOW_ERROR,
)
from .models import OperationResult, RobotPose
from .waypoints import WaypointExecutor

__all__ = [
    "ERROR_BUSY",
    "ERROR_CANCELLED",
    "ERROR_FAULTED",
    "ERROR_GOAL_REJECTED",
    "ERROR_GRIPPER_FAILED",
    "ERROR_MOVE_FAILED",
    "ERROR_OUT_OF_WORKSPACE",
    "ERROR_SERVER_UNAVAILABLE",
    "ERROR_TIMEOUT",
    "ERROR_WORKFLOW_BUSY",
    "ERROR_WORKFLOW_ERROR",
    "OperationResult",
    "RobotPose",
    "WaypointExecutor",
]
