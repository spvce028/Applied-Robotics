
from dataclasses import dataclass
from typing import Optional


@dataclass
class RobotPose:
    """Target pose relative to the robot base.

    Position is measured in millimeters.
    Rotation r is measured in degrees.
    """

    x: float
    y: float
    z: float
    r: float
    frame_id: str = "base_link"


@dataclass
class OperationResult:
    """Result of a robot or gripper operation."""

    success: bool
    message: str = ""
    error_code: Optional[str] = None
