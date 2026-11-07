import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dobot_msgs.action import PointToPoint
from depalletizing_interfaces.models import RobotPose, OperationResult


class DobotActionRobot:
    """ROS 2 Action Client wrapper for the Dobot Magician / jkaniuka stack."""

    def __init__(self, node: Node):
        self.node = node
        self._action_client = ActionClient(self.node, PointToPoint, '/PTP_action')
        
        self.node.get_logger().info("Waiting for Dobot PTP Action Server...")
        self._action_client.wait_for_server()
        self.node.get_logger().info("Dobot Action Server connected!")

    def move_to(self, pose: RobotPose, motion_type: int = 1) -> OperationResult:
        """Sends a PointToPoint action goal, monitors status via feedback, and handles errors."""
        goal_msg = PointToPoint.Goal()
        goal_msg.motion_type = motion_type  # 1: Joint interpolated motion (Cartesian)
        goal_msg.target_pose = [float(pose.x), float(pose.y), float(pose.z), float(pose.r)]
        goal_msg.velocity_ratio = 0.4
        goal_msg.acceleration_ratio = 0.4

        # Send goal synchronously / blocking for the high-level primitive workflow
        # (Or use a future-spinning approach compatible with rclpy)
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            return OperationResult(
                success=False,
                message="Goal rejected by Dobot action server (workspace limit or conflict)."
            )

        # Wait for the actual result execution
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, get_result_future)
        
        result_wrapped = get_result_future.result()
        status = result_wrapped.status

        # ROS 2 Action Status: 4 = STATUS_SUCCEEDED
        if status == 4:
            return OperationResult(success=True, message="Motion completed successfully.")
        else:
            error_msg = f"Motion failed or aborted with status code: {status}"
            self.node.get_logger().error(error_msg)
            self._handle_error_recovery(status)
            return OperationResult(success=False, message=error_msg)

    def _feedback_callback(self, feedback_msg):
        """Movement Status Monitoring callback."""
        feedback = feedback_msg.feedback
        # Logs status updates coming from the hardware driver loop
        self.node.get_logger().info("[Status Monitor] Dobot movement in progress...")

    def _handle_error_recovery(self, status_code):
        """Basic error handling recovery routine."""
        self.node.get_logger().warn(f"Executing recovery protocol for error state: {status_code}")
        # Add safety actions here (e.g., clear alarms or safe pause)