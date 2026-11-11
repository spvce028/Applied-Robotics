import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future

from dobot_msgs.action import PointToPoint
from action_msgs.msg import GoalStatus

from depalletizing_interfaces.models import RobotPose, OperationResult


class DobotActionRobot:
    """
    Synchronous ROS 2 Action Client wrapper for the Dobot Magician.
    Sends PointToPoint goals and processes acceptance, feedback, 
    and execution results completely asynchronously via futures and callbacks.
    """

    def __init__(self, node: Node):
        self.node = node
        self._action_client = ActionClient(self.node, PointToPoint, '/PTP_action')
        
        self._active_goal_handle = None
        self._goal_pending = False
        self._lock_future = None
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

    def move_to(self, pose: RobotPose, motion_type: int = 1) -> OperationResult:
        """
        Send a movement goal and block until completion, cancellation, or timeout.
        Returns:
            OperationResult: Direct result object evaluated upon motion completion.
        """
        timeout_sec = 30.0

        # Prevent overlapping movement commands.
        if self._goal_pending or self._active_goal_handle is not None:
            return OperationResult(success=False, message="Another robot movement is already active.")

        if not self._action_client.server_is_ready():
            return OperationResult(success=False, message="Dobot Action Server is not available.")

        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = motion_type
        goal_msg.target_pose = [float(pose.x), float(pose.y), float(pose.z), float(pose.r)]
        goal_msg.velocity_ratio = 0.4
        goal_msg.acceleration_ratio = 0.4

        self.node.get_logger().info(
            f"Sending Dobot movement goal: x={pose.x}, y={pose.y}, z={pose.z}, r={pose.r}"
        )

        self._goal_pending = True
        result_future = Future()
        self._lock_future = result_future

        def goal_response_callback(future):
            if result_future.done():
                return  # Already timed out or completed
            try:
                goal_handle = future.result()
            except Exception as exc:
                self._goal_pending = False
                result_future.set_result(OperationResult(success=False, message=f"Goal response failed: {exc}"))
                return

            if not goal_handle.accepted:
                self._goal_pending = False
                result_future.set_result(OperationResult(success=False, message="Dobot rejected the movement goal."))
                return

            self._active_goal_handle = goal_handle
            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_callback)

        def result_callback(future):
            if result_future.done():
                return  # Already handled (e.g. timeout)
            self._goal_pending = False
            self._active_goal_handle = None
            try:
                result_wrapped = future.result()
                status = result_wrapped.status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    res = OperationResult(success=True, message="Motion completed successfully.")
                elif status == GoalStatus.STATUS_CANCELED:
                    res = OperationResult(success=False, message="Motion was canceled.")
                else:
                    res = OperationResult(success=False, message=f"Motion failed with status: {status}")
                result_future.set_result(res)
            except Exception as exc:
                result_future.set_result(OperationResult(success=False, message=f"Failed to retrieve result: {exc}"))


        try:
            send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self._feedback_callback
            )

            send_goal_future.add_done_callback(goal_response_callback)

        except Exception as exc:
            self._goal_pending = False
            result_future.set_result(
                OperationResult(
                    success=False,
                    message=f"Failed to send movement goal: {exc}"
                )
            )

        # Spin with strict execution timeout guard
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_sec)

        # Handle timeout case if execution takes longer than 30 seconds
        if not result_future.done():
            self.node.get_logger().error("Dobot motion execution timed out after 30 seconds! Cancelling goal...")
            self._goal_pending = False

            if self._active_goal_handle is not None:
                try:
                    cancel_future = self._active_goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=2.0)
                except Exception as e:
                    self.node.get_logger().error(f"Failed to cleanly cancel goal: {e}")
                finally:
                    self._active_goal_handle = None

            timeout_result = OperationResult(success=False, message="Motion execution timed out after 30 seconds.")
            result_future.set_result(timeout_result)

        return result_future.result()

    def _feedback_callback(self, feedback_msg):
        pass

    def cancel_current_goal(self):
        if self._active_goal_handle is None:
            return None
        return self._active_goal_handle.cancel_goal_async()