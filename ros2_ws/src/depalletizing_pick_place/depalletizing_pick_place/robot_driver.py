"""Asynchronous Dobot Magician ROS 2 action client."""

import threading

from rclpy.clock import Clock, ClockType

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from dobot_msgs.action import PointToPoint
from action_msgs.msg import GoalStatus

from depalletizing_interfaces.models import RobotPose, OperationResult


class DobotActionRobot:
    """Asynchronous ROS 2 Action Client for Dobot PTP movements."""

    @property
    def is_faulted(self) -> bool:
        with self._state_lock:
            return self._faulted

    def __init__(self, node: Node):
        self.node = node
        self._action_client = ActionClient(self.node, PointToPoint, '/PTP_action')
        
        self._active_operation = None

        self._faulted = False
        self._fault_reason = None

        self._state_lock = threading.RLock()
        self.node.get_logger().info("Dobot Action Client initialized.")

    def wait_for_server(self, timeout_sec=5.0) -> bool:
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

    def move_to(self, pose: RobotPose, motion_type: int = 1) -> Future:
        """
        Send a movement goal asynchronously.
        Returns:
            Future containing an OperationResult.
        """
        result_future = Future()

        with self._state_lock:

            if self._faulted:
                result_future.set_result(
                    OperationResult(
                        success=False,
                        message=f"Robot driver faulted: {self._fault_reason}"
                    )
                )
                return result_future

            # Prevent overlapping movement commands.
            if self._active_operation is not None:
                result_future.set_result(
                    OperationResult(
                        success=False,
                        message="Another robot movement is already active."
                    )
                )
                return result_future

            if not self._action_client.server_is_ready():
                result_future.set_result(
                    OperationResult(
                        success=False,
                        message="Dobot Action Server is unavailable."
                    )
                )
                return result_future

            operation = {
                "future": result_future,
                "goal_handle": None,
                "timed_out": False,
                "cancel_requested": False,
                "timeout_timer": None,
            }

            # Lock the driver before sending the goal.
            self._active_operation = operation

            # Start the timeout timer.
            operation["timeout_timer"] = self.node.create_timer(
                30.0,
                lambda: self._handle_timeout(operation),
                clock=Clock(clock_type=ClockType.STEADY_TIME)
            )

        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = motion_type
        goal_msg.target_pose = [float(pose.x), float(pose.y), float(pose.z), float(pose.r)]
        goal_msg.velocity_ratio = 0.4
        goal_msg.acceleration_ratio = 0.4

        self.node.get_logger().info(
            f"Sending Dobot movement goal: x={pose.x}, y={pose.y}, z={pose.z}, r={pose.r}"
        )

        try:
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )

            send_goal_future.add_done_callback(
                lambda future: self._goal_response_callback(
                    future, operation
                )
            )

        except Exception as exc:
            self._enter_fault(
                operation,
                f"Unable to confirm goal submission: {exc}"
            )

        return result_future

    def _goal_response_callback(self, future, operation):

        try:
            goal_handle = future.result()

        except Exception as exc:
            self._enter_fault(
                operation,
                f"Unable to determine whether Dobot accepted the goal: {exc}"
            )
            return

        if not goal_handle.accepted:

            message = (
                "Movement timed out before goal acceptance."
                if operation["timed_out"]
                else "Dobot rejected the movement goal."
            )

            self._finish_operation(
                operation,
                OperationResult(
                    success=False,
                    message=message
                )
            )
            return

        with self._state_lock:

            # Store the handle even if the operation timed out.
            # A late-accepted goal must still be cancelled.
            operation["goal_handle"] = goal_handle

            should_cancel = (
                operation["timed_out"]
                or operation["cancel_requested"]
            )

        self.node.get_logger().info(
            "Dobot movement goal accepted."
        )

        try:
            result_future = goal_handle.get_result_async()

            result_future.add_done_callback(
                lambda future: self._result_callback(
                    future, operation
                )
            )

        except Exception as exc:
            self._enter_fault(
                operation,
                f"Failed to register movement result callback: {exc}"
            )
            return

        if should_cancel:
            self._request_goal_cancellation(operation)

    def _handle_timeout(self, operation):

        with self._state_lock:

            if self._active_operation is not operation:
                return

            if operation["timed_out"]:
                return

            operation["timed_out"] = True

            goal_handle = operation["goal_handle"]

            self.node.get_logger().error(
                "Dobot movement timed out after 30 seconds."
            )

            # Report timeout to the workflow immediately.
            # Keep the driver locked until a terminal action result.
            if not operation["future"].done():
                operation["future"].set_result(
                    OperationResult(
                        success=False,
                        message="Motion execution timed out after 30 seconds."
                    )
                )

        # If acceptance is still pending, the goal response callback
        # will cancel the goal if it is eventually accepted.
        if goal_handle is not None:
            self._request_goal_cancellation(operation)

    def _request_goal_cancellation(self, operation):

        with self._state_lock:

            if self._active_operation is not operation:
                return

            goal_handle = operation["goal_handle"]

            if goal_handle is None:
                operation["cancel_requested"] = True
                return

            if operation.get("cancellation_sent", False):
                return

            operation["cancellation_sent"] = True

        self.node.get_logger().warning(
            "Requesting Dobot movement cancellation..."
        )

        try:
            cancel_future = goal_handle.cancel_goal_async()

            cancel_future.add_done_callback(
                lambda future: self._cancel_response_callback(
                    future, operation
                )
            )

        except Exception as exc:
            self._enter_fault(
                operation,
                f"Cancellation request failed: {exc}"
            )

    def _cancel_response_callback(self, future, operation):

        try:
            response = future.result()

            if response.goals_canceling:
                self.node.get_logger().warning(
                    "Dobot accepted cancellation request. "
                    "Waiting for terminal action result."
                )
            else:
                self._enter_fault(
                    operation,
                    "Dobot rejected the cancellation request. "
                    "Robot movement state is uncertain."
                )

        except Exception as exc:
            self._enter_fault(
                operation,
                f"Cancellation response failed: {exc}"
            )
            

    def _result_callback(self, future, operation):

        try:
            result_wrapped = future.result()
            status = result_wrapped.status

            if status == GoalStatus.STATUS_SUCCEEDED:

                result = OperationResult(
                    success=True,
                    message="Motion completed successfully."
                )

            elif status == GoalStatus.STATUS_CANCELED:

                result = OperationResult(
                    success=False,
                    message="Motion was canceled."
                )

            else:

                result = OperationResult(
                    success=False,
                    message=f"Motion failed with status: {status}"
                )

        except Exception as exc:
            self._enter_fault(
                operation,
                f"Unable to retrieve terminal movement result: {exc}"
            )
            return

        if operation["timed_out"]:
            result = OperationResult(
                success=False,
                message="Movement timed out. Terminal action result received."
            )

        # A terminal result confirms that the action is no longer active.
        self._finish_operation(operation, result)

    def _finish_operation(self, operation, result):

        with self._state_lock:

            if self._active_operation is not operation:
                return

            timer = operation["timeout_timer"]

            if timer is not None:
                timer.cancel()
                self.node.destroy_timer(timer)

            self._active_operation = None

            # Never overwrite a previously reported timeout.
            if not operation["future"].done():
                operation["future"].set_result(result)

        self.node.get_logger().info(
            f"Movement operation finished: {result.message}"
        )

    def _feedback_callback(self, feedback_msg):
        """Process Dobot action feedback."""
        pass

    def cancel_current_goal(self):
        """Request cancellation of the current movement."""

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

    def _enter_fault(self, operation, reason):
        """Keep the robot locked after an uncertain action state."""

        with self._state_lock:

            if self._active_operation is not operation:
                return

            self._faulted = True
            self._fault_reason = reason

            self.node.get_logger().fatal(
                f"ROBOT DRIVER FAULT: {reason}"
            )

            # Notify the workflow, but retain the movement lock.
            if not operation["future"].done():
                operation["future"].set_result(
                    OperationResult(
                        success=False,
                        message=f"Robot driver faulted: {reason}"
                    )
                )

    