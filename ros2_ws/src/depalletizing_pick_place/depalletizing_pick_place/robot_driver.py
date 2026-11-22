"""Asynchronous Dobot Magician ROS 2 action client.

Driver wraps PointToPoint action server (``/PTP_action`` by default)
and exposes single fully asynchronous entry point,
:meth:`DobotActionRobot.move_to`.  Every code path resolves returned
future with :class:`~depalletizing_interfaces.models.OperationResult` and
no call blocks ROS executor.

Safety model
------------
Movement locks driver. Lock is only released once action
reaches *terminal* state (succeeded, aborted, cancelled, or rejected).  
On timeout driver reports ``TIMEOUT`` immediately, requests cancellation of
active goal, but stays locked so that no second movement can be issued
while first goal may still be moving.  If robot state becomes uncertain
driver enters fault; use :meth:`reset_fault` (after verifying robot is safe) 
or restart node to recover.
"""

import math
import threading
from dataclasses import dataclass

from action_msgs.msg import GoalStatus
from dobot_msgs.action import PointToPoint
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future

from depalletizing_interfaces.errors import (
    ERROR_BUSY,
    ERROR_CANCELLED,
    ERROR_FAULTED,
    ERROR_GOAL_REJECTED,
    ERROR_MOVE_FAILED,
    ERROR_OUT_OF_WORKSPACE,
    ERROR_SERVER_UNAVAILABLE,
    ERROR_TIMEOUT,
)
from depalletizing_interfaces.models import OperationResult, RobotPose


@dataclass(frozen=True)
class WorkspaceLimits:
    """Axis-aligned Cartesian limits in millimetres and degrees.

    Defaults describe a conservative Dobot Magician envelope. They are a
    software guard only: verify them against the real cell before hardware
    testing and pass measured values through ``workspace_limits``.
    """

    x: tuple[float, float] = (-320.0, 320.0)
    y: tuple[float, float] = (-320.0, 320.0)
    z: tuple[float, float] = (-100.0, 300.0)
    r: tuple[float, float] = (-360.0, 360.0)

    def as_dict(self) -> dict[str, tuple[float, float]]:
        """Return the limits keyed by pose axis."""
        return {"x": self.x, "y": self.y, "z": self.z, "r": self.r}

    def __post_init__(self):
        """Reject malformed bounds at construction time."""
        for axis, bounds in self.as_dict().items():
            if len(bounds) != 2 or not all(math.isfinite(v) for v in bounds):
                raise ValueError(f"{axis} limits must be two finite numbers")
            if bounds[0] > bounds[1]:
                raise ValueError(f"{axis} lower limit exceeds upper limit")

    def violations(self, pose: RobotPose) -> list[str]:
        """Return a description of every axis outside the limits."""
        values = {"x": pose.x, "y": pose.y, "z": pose.z, "r": pose.r}
        problems = []
        for axis, (lower, upper) in self.as_dict().items():
            value = values[axis]
            if not math.isfinite(value):
                problems.append(f"{axis} is not a finite number")
            elif value < lower or value > upper:
                problems.append(f"{axis}={value} outside [{lower}, {upper}]")
        return problems


DEFAULT_WORKSPACE_LIMITS = WorkspaceLimits()


class DobotActionRobot:
    """Asynchronous ROS 2 action client for Dobot PTP movements."""

    DEFAULT_ACTION_SERVER = "/PTP_action"
    DEFAULT_VELOCITY_RATIO = 0.4
    DEFAULT_ACCELERATION_RATIO = 0.4
    DEFAULT_MOVEMENT_TIMEOUT = 30.0

    def __init__(
        self,
        node: Node,
        action_server_name: str = DEFAULT_ACTION_SERVER,
        velocity_ratio: float = DEFAULT_VELOCITY_RATIO,
        acceleration_ratio: float = DEFAULT_ACCELERATION_RATIO,
        movement_timeout: float = DEFAULT_MOVEMENT_TIMEOUT,
        workspace_limits: WorkspaceLimits | None = DEFAULT_WORKSPACE_LIMITS,
    ):
        """Create the action client.

        Args:
            node: ROS node used for timers, logging and the action client.
            action_server_name: Name of the PointToPoint action server.
            velocity_ratio: Motion velocity ratio passed to the server.
            acceleration_ratio: Motion acceleration ratio passed to the server.
            movement_timeout: Seconds before an active goal is considered
                timed out and cancellation is requested.
            workspace_limits: Cartesian bounds checked before every goal.
                Pass ``None`` to use the conservative defaults.
        """
        if not 0.0 < velocity_ratio <= 1.0:
            raise ValueError("velocity_ratio must be within (0.0, 1.0]")
        if not 0.0 < acceleration_ratio <= 1.0:
            raise ValueError("acceleration_ratio must be within (0.0, 1.0]")
        if movement_timeout <= 0.0:
            raise ValueError("movement_timeout must be positive")
        if workspace_limits is None:
            workspace_limits = DEFAULT_WORKSPACE_LIMITS
        if not callable(getattr(workspace_limits, "violations", None)):
            raise ValueError("workspace_limits must provide violations(pose)")

        self.node = node
        self.action_server_name = action_server_name
        self.velocity_ratio = velocity_ratio
        self.acceleration_ratio = acceleration_ratio
        self.movement_timeout = movement_timeout
        self.workspace_limits = workspace_limits

        self._action_client = ActionClient(
            self.node, PointToPoint, action_server_name
        )

        self._active_operation = None
        self._faulted = False
        self._fault_reason = None
        self._state_lock = threading.RLock()

        self.node.get_logger().info(
            f"Dobot Action Client initialized for '{action_server_name}'."
        )

    # ------------------------------------------------------------------ state
    @property
    def is_faulted(self) -> bool:
        """Return True while the driver is locked in a fault state."""
        with self._state_lock:
            return self._faulted

    @property
    def fault_reason(self):
        """Return the reason for the current fault, if any."""
        with self._state_lock:
            return self._fault_reason

    @property
    def is_busy(self) -> bool:
        """Return True while a movement is active or awaiting a terminal result."""
        with self._state_lock:
            return self._active_operation is not None

    @property
    def last_feedback(self):
        """Return the most recent processed movement feedback, if any."""
        with self._state_lock:
            operation = self._active_operation
            if operation is None:
                return None
            return operation.get("last_feedback")

    # --------------------------------------------------------------- server
    def wait_for_server(self, timeout_sec: float = 5.0) -> bool:
        """Wait for the Dobot action server during initialization."""
        self.node.get_logger().info("Waiting for Dobot PTP Action Server...")

        available = self._action_client.wait_for_server(timeout_sec=timeout_sec)

        if available:
            self.node.get_logger().info("Dobot Action Server connected!")
        else:
            self.node.get_logger().error("Dobot Action Server not available.")
        return available

    def server_is_ready(self) -> bool:
        """Check whether the action server is available."""
        return self._action_client.server_is_ready()

    def is_pose_within_workspace(self, pose: RobotPose) -> bool:
        """Return True when every pose axis is inside the configured limits."""
        return not self.workspace_limits.violations(pose)

    # -------------------------------------------------------------- movement
    def move_to(self, pose: RobotPose, motion_type: int = 1) -> Future:
        """Send a movement goal asynchronously.

        This method never blocks: it returns immediately with a Future that is
        resolved with an OperationResult once the movement reaches a terminal
        state (or is rejected/timed out).

        Args:
            pose: Target pose for the robot, validated against workspace limits.
            motion_type: PointToPoint motion type (1 = MOVJ in Cartesian space,
                2 = MOVL linear Cartesian).

        Returns:
            Future resolved with an OperationResult. An out-of-workspace pose
            resolves immediately with ``OUT_OF_WORKSPACE``.
        """
        result_future = Future()

        # Validate before touching hardware or taking the movement lock.
        violations = self.workspace_limits.violations(pose)
        if violations:
            message = (
                "Target pose outside workspace limits: "
                + "; ".join(violations)
            )
            self.node.get_logger().error(message)
            result_future.set_result(
                OperationResult(
                    success=False,
                    message=message,
                    error_code=ERROR_OUT_OF_WORKSPACE,
                )
            )
            return result_future

        with self._state_lock:
            if self._faulted:
                result_future.set_result(
                    OperationResult(
                        success=False,
                        message=f"Robot driver faulted: {self._fault_reason}",
                        error_code=ERROR_FAULTED,
                    )
                )
                return result_future

            # Prevent overlapping movement commands.
            if self._active_operation is not None:
                result_future.set_result(
                    OperationResult(
                        success=False,
                        message="Another robot movement is already active.",
                        error_code=ERROR_BUSY,
                    )
                )
                return result_future

            if not self._action_client.server_is_ready():
                result_future.set_result(
                    OperationResult(
                        success=False,
                        message="Dobot Action Server is unavailable.",
                        error_code=ERROR_SERVER_UNAVAILABLE,
                    )
                )
                return result_future

            operation = {
                "future": result_future,
                "goal_handle": None,
                "timed_out": False,
                "cancel_requested": False,
                "cancellation_sent": False,
                "timeout_timer": None,
                "last_feedback": None,
            }

            # Lock the driver before sending the goal.
            self._active_operation = operation

            # Start the timeout timer.  A steady clock keeps the timeout
            # working even when simulated ROS time does not advance.
            operation["timeout_timer"] = self.node.create_timer(
                self.movement_timeout,
                lambda: self._handle_timeout(operation),
                clock=Clock(clock_type=ClockType.STEADY_TIME),
            )

        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = motion_type
        goal_msg.target_pose = [
            float(pose.x),
            float(pose.y),
            float(pose.z),
            float(pose.r),
        ]
        goal_msg.velocity_ratio = self.velocity_ratio
        goal_msg.acceleration_ratio = self.acceleration_ratio

        self.node.get_logger().info(
            f"Sending Dobot movement goal: x={pose.x}, y={pose.y}, "
            f"z={pose.z}, r={pose.r}"
        )

        try:
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=lambda feedback: self._feedback_callback(
                    feedback, operation
                ),
            )

            send_goal_future.add_done_callback(
                lambda future: self._goal_response_callback(future, operation)
            )

        except Exception as exc:
            self._enter_fault(
                operation,
                f"Unable to submit movement goal: {exc}",
            )

        return result_future

    def cancel_current_goal(self):
        """Request cancellation of the current movement.

        Returns True when a cancellation request was sent, or None when no
        movement is active or acceptance is still pending.
        """
        with self._state_lock:
            operation = self._active_operation

            if operation is None:
                return None

            operation["cancel_requested"] = True

            if operation["goal_handle"] is None:
                self.node.get_logger().warning(
                    "Cancellation requested while waiting for goal acceptance."
                )
                return None

        self._request_goal_cancellation(operation)
        return True

    # -------------------------------------------------------------- callbacks
    def _goal_response_callback(self, future, operation):
        """Handle the action server's decision to accept or reject the goal."""
        try:
            goal_handle = future.result()

        except Exception as exc:
            self._enter_fault(
                operation,
                f"Unable to determine whether Dobot accepted the goal: {exc}",
            )
            return

        with self._state_lock:
            stale = self._active_operation is not operation

        if stale:
            # The operation was already abandoned (for example after a forced
            # reset). Make sure a late-accepted goal cannot keep moving
            # unnoticed.
            if getattr(goal_handle, "accepted", False):
                self.node.get_logger().warning(
                    "Cancelling a goal that was accepted after the operation "
                    "had already ended."
                )
                try:
                    goal_handle.cancel_goal_async()
                except Exception as exc:
                    self.node.get_logger().error(
                        f"Failed to cancel the abandoned goal: {exc}"
                    )
            return

        if not goal_handle.accepted:
            with self._state_lock:
                timed_out = operation["timed_out"]

            if timed_out:
                self._finish_operation(
                    operation,
                    OperationResult(
                        success=False,
                        message="Movement timed out before the goal was accepted.",
                        error_code=ERROR_TIMEOUT,
                    ),
                )
            else:
                self.node.get_logger().error("Dobot rejected the movement goal.")
                self._finish_operation(
                    operation,
                    OperationResult(
                        success=False,
                        message="Dobot rejected the movement goal.",
                        error_code=ERROR_GOAL_REJECTED,
                    ),
                )
            return

        with self._state_lock:
            # Store the handle even if the operation timed out; a
            # late-accepted goal must still be cancelled.
            operation["goal_handle"] = goal_handle
            should_cancel = (
                operation["timed_out"] or operation["cancel_requested"]
            )

        self.node.get_logger().info("Dobot movement goal accepted.")

        try:
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda future: self._result_callback(future, operation)
            )
        except Exception as exc:
            self._enter_fault(
                operation,
                f"Failed to register the movement result callback: {exc}",
            )
            return

        if should_cancel:
            self._request_goal_cancellation(operation)

    def _handle_timeout(self, operation):
        """Report a timeout, cancel the goal, and keep the driver locked."""
        with self._state_lock:
            if self._active_operation is not operation:
                return
            if operation["timed_out"]:
                return

            operation["timed_out"] = True
            goal_handle = operation["goal_handle"]

            timer = operation["timeout_timer"]
            if timer is not None:
                timer.cancel()

            self.node.get_logger().error(
                f"Dobot movement timed out after {self.movement_timeout} seconds."
            )

            # Report the timeout immediately, but keep the driver locked until
            # a terminal action result confirms the robot has stopped.
            if not operation["future"].done():
                operation["future"].set_result(
                    OperationResult(
                        success=False,
                        message=(
                            "Motion execution timed out after "
                            f"{self.movement_timeout} seconds."
                        ),
                        error_code=ERROR_TIMEOUT,
                    )
                )

        # If acceptance is still pending, the goal response callback will
        # cancel the goal once it is eventually accepted.
        if goal_handle is not None:
            self._request_goal_cancellation(operation)

    def _request_goal_cancellation(self, operation):
        """Ask the action server to cancel the active goal."""
        with self._state_lock:
            if self._active_operation is not operation:
                return

            goal_handle = operation["goal_handle"]

            if goal_handle is None:
                operation["cancel_requested"] = True
                return

            if operation["cancellation_sent"]:
                return

            operation["cancellation_sent"] = True

        self.node.get_logger().warning(
            "Requesting Dobot movement cancellation..."
        )

        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda future: self._cancel_response_callback(future, operation)
            )
        except Exception as exc:
            self._enter_fault(
                operation,
                f"Cancellation request failed: {exc}",
            )

    def _cancel_response_callback(self, future, operation):
        """Handle the action server's response to a cancellation request."""
        try:
            response = future.result()
        except Exception as exc:
            self._enter_fault(
                operation,
                f"Cancellation response failed: {exc}",
            )
            return

        if getattr(response, "goals_canceling", None):
            self.node.get_logger().warning(
                "Dobot accepted the cancellation request. "
                "Waiting for the terminal action result."
            )
        else:
            self._enter_fault(
                operation,
                "Dobot rejected the cancellation request. "
                "Robot movement state is uncertain.",
            )

    def _result_callback(self, future, operation):
        """Handle the terminal action result."""
        try:
            result_wrapped = future.result()
            status = result_wrapped.status
        except Exception as exc:
            self._enter_fault(
                operation,
                f"Unable to retrieve the terminal movement result: {exc}",
            )
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            result = OperationResult(
                success=True,
                message="Motion completed successfully.",
            )
        elif status == GoalStatus.STATUS_CANCELED:
            result = OperationResult(
                success=False,
                message="Motion was canceled.",
                error_code=ERROR_CANCELLED,
            )
        elif status == GoalStatus.STATUS_ABORTED:
            result = OperationResult(
                success=False,
                message="Motion was aborted by the action server.",
                error_code=ERROR_MOVE_FAILED,
            )
        else:
            result = OperationResult(
                success=False,
                message=f"Motion failed with status: {status}",
                error_code=ERROR_MOVE_FAILED,
            )

        with self._state_lock:
            timed_out = operation["timed_out"]

        if timed_out:
            result = OperationResult(
                success=False,
                message="Movement timed out. Terminal action result received.",
                error_code=ERROR_TIMEOUT,
            )

        # A terminal result confirms that the action is no longer active.
        self._finish_operation(operation, result)

    def _finish_operation(self, operation, result):
        """Release the movement lock and resolve the result future once."""
        with self._state_lock:
            if self._active_operation is not operation:
                return

            timer = operation["timeout_timer"]
            if timer is not None:
                timer.cancel()
                self.node.destroy_timer(timer)
                operation["timeout_timer"] = None

            self._active_operation = None

            # Never overwrite a result that was already reported (e.g. a
            # timeout), and never resolve a future twice.
            if not operation["future"].done():
                operation["future"].set_result(result)

        self.node.get_logger().info(
            f"Movement operation finished: {result.message}"
        )

    def _feedback_callback(self, feedback_msg, operation=None):
        """Process and log meaningful movement feedback."""
        feedback = getattr(feedback_msg, "feedback", None)
        current_pose = getattr(feedback, "current_pose", None)

        with self._state_lock:
            if operation is not None and self._active_operation is not operation:
                return

            if current_pose is not None:
                current_pose = [float(value) for value in current_pose]

            previous = None
            if operation is not None:
                previous = operation.get("last_feedback")
                operation["last_feedback"] = current_pose

        if current_pose is None:
            self.node.get_logger().debug("Dobot movement feedback received.")
        elif current_pose != previous:
            self.node.get_logger().debug(
                f"Dobot movement feedback: current_pose={current_pose}"
            )

    # -------------------------------------------------------------- recovery
    def _enter_fault(self, operation, reason):
        """Keep the robot locked after an uncertain action state."""
        with self._state_lock:
            if self._active_operation is not operation:
                return

            self._faulted = True
            self._fault_reason = reason

            self.node.get_logger().fatal(f"ROBOT DRIVER FAULT: {reason}")

            # Notify the workflow, but retain the movement lock.
            if not operation["future"].done():
                operation["future"].set_result(
                    OperationResult(
                        success=False,
                        message=f"Robot driver faulted: {reason}",
                        error_code=ERROR_FAULTED,
                    )
                )

    def reset_fault(self, force: bool = False) -> bool:
        """Attempt to clear a fault so new movements can be issued.

        Plain reset refused while movement may still be active; verify
        robot safe (or restart node) first.  ``force=True`` clears
        lock unconditionally and must only be used once robot has been
        verified to be stationary.

        Returns:
            True when the driver is no longer faulted.
        """
        with self._state_lock:
            if not self._faulted:
                return True

            if self._active_operation is not None and not force:
                self.node.get_logger().error(
                    "Cannot reset robot driver while movement may still "
                    "be active. Restart node, or call "
                    "reset_fault(force=True) after verifying robot is safe."
                )
                return False

            operation = self._active_operation
            if operation is not None:
                timer = operation["timeout_timer"]
                if timer is not None:
                    timer.cancel()
                    self.node.destroy_timer(timer)
                    operation["timeout_timer"] = None
                self._active_operation = None

            self._faulted = False
            self._fault_reason = None

        self.node.get_logger().warning(
            "Robot driver fault cleared. Verify state before next movement."
        )
        return True
