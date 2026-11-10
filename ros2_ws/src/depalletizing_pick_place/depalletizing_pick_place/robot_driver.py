import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from dobot_msgs.action import PointToPoint
from action_msgs.msg import GoalStatus

from depalletizing_interfaces.models import RobotPose, OperationResult


class DobotActionRobot:
    """
    Asynchronous ROS 2 Action Client wrapper for the Dobot Magician.
    Sends PointToPoint goals and processes acceptance, feedback, 
    and execution results completely asynchronously via futures and callbacks.
    """

    def __init__(self, node: Node):
        self.node = node
        self._action_client = ActionClient(self.node, PointToPoint, '/PTP_action')
        
        self._active_goal_handle = None
        self._goal_pending = False
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

    def move_to(self, pose: RobotPose, motion_type: int = 1) -> Future:
        """
        Send an asynchronous movement goal.
        Returns:
            Future[OperationResult]: Completes when the robot finishes execution.
        """
        result_future = Future()

        # Prevent overlapping movement commands.
        if self._goal_pending or self._active_goal_handle is not None:
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
                    message="Dobot Action Server is not available."
                )
            )
            return result_future

        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = motion_type
        goal_msg.target_pose = [float(pose.x), float(pose.y), float(pose.z), float(pose.r)]
        goal_msg.velocity_ratio = 0.4
        goal_msg.acceleration_ratio = 0.4

        self.node.get_logger().info(
            f"Sending Dobot movement goal: x={pose.x}, y={pose.y}, z={pose.z}, r={pose.r}"
        )

        self._goal_pending = True

        try:
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )

            send_goal_future.add_done_callback(
                lambda future: self._goal_response_callback(future, result_future)
            )

        except Exception as exc:
            self._goal_pending = False
            result_future.set_result(
                OperationResult(
                    success=False,
                    message=f"Failed to send movement goal: {exc}"
                )
            )

        return result_future

    def _goal_response_callback(self, future, result_future):
        """Handle goal acceptance or rejection."""
        self._goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            result_future.set_result(
                OperationResult(success=False, message=f"Goal response failed: {exc}")
            )
            return

        if not goal_handle.accepted:
            self.node.get_logger().warning("Dobot rejected the movement goal.")
            result_future.set_result(
                OperationResult(success=False, message="Dobot rejected the movement goal.")
            )
            return

        self.node.get_logger().info("Movement goal accepted. Robot is moving.")
        self._active_goal_handle = goal_handle

        try:
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(
                lambda future: self._result_callback(future, result_future, goal_handle)
            )
        except Exception as exc:
            result_future.set_result(
                OperationResult(success=False, message=f"Failed to request result: {exc}")
            )

    def _feedback_callback(self, feedback_msg):
        """Movement Status Monitoring callback."""
        self.node.get_logger().debug("Dobot movement feedback received.")

    def _result_callback(self, future, result_future, goal_handle):
        """Handle the final movement result."""
        try:
            result_wrapped = future.result()
            status = result_wrapped.status

            if status == GoalStatus.STATUS_SUCCEEDED:
                operation_result = OperationResult(success=True, message="Motion completed successfully.")
            elif status == GoalStatus.STATUS_CANCELED:
                operation_result = OperationResult(success=False, message="Motion was canceled.")
            else:
                operation_result = OperationResult(success=False, message=f"Motion failed with status: {status}")

            result_future.set_result(operation_result)

        except Exception as exc:
            result_future.set_result(
                OperationResult(success=False, message=f"Failed to retrieve result: {exc}")
            )
        finally:
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None

    def cancel_current_goal(self):
        """Request cancellation of the currently active movement."""
        if self._active_goal_handle is None:
            self.node.get_logger().warning("No active movement to cancel.")
            return None

        self.node.get_logger().warning("Requesting Dobot movement cancellation.")
        return self._active_goal_handle.cancel_goal_async()