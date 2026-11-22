"""Fake Dobot PointToPoint action server for simulation.

This node implements the same ``dobot_msgs/action/PointToPoint`` interface as
the real driver. It accepts a goal, publishes interpolated ``current_pose``
feedback, and finishes successfully after a short simulated movement. It also
honours cancellation requests so the whole driver workflow can be exercised
without hardware.

The execute callback follows the rclpy contract: it returns a
``PointToPoint.Result`` message. It runs in an executor worker thread, so a
``MultiThreadedExecutor`` is required to process cancel requests while a
movement is simulated.
"""

import math
import threading
import time

from dobot_msgs.action import PointToPoint
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import rclpy


SUPPORTED_MOTION_TYPES = (1, 2, 4, 5)


def interpolate_pose(start, target, progress):
    """Return a linearly interpolated ``[x, y, z, r]`` pose.

    ``progress`` is clamped to ``[0.0, 1.0]`` so a late tick cannot overshoot
    the target.
    """
    ratio = max(0.0, min(1.0, float(progress)))
    return [begin + (end - begin) * ratio for begin, end in zip(start, target)]


class FakePTPActionServer(Node):
    """Fake PointToPoint action server with timer-paced simulated motion."""

    def __init__(
        self,
        action_name: str = "/PTP_action",
        feedback_interval: float = 0.1,
        movement_duration: float = 1.0,
        start_pose=None,
    ):
        """Create the fake action server.

        Args:
            action_name: Name of the PointToPoint action.
            feedback_interval: Seconds between feedback messages.
            movement_duration: Simulated duration of a full movement.
            start_pose: Initial ``[x, y, z, r]`` pose, defaulting to origin.
        """
        super().__init__("fake_ptp_action_server")

        # Constructor values act as defaults; ROS parameters can override them.
        self.declare_parameter("action_name", action_name)
        self.declare_parameter("feedback_interval", feedback_interval)
        self.declare_parameter("movement_duration", movement_duration)
        action_name = self.get_parameter("action_name").value
        feedback_interval = float(
            self.get_parameter("feedback_interval").value
        )
        movement_duration = float(
            self.get_parameter("movement_duration").value
        )

        if feedback_interval <= 0.0:
            raise ValueError("feedback_interval must be positive")
        if movement_duration <= 0.0:
            raise ValueError("movement_duration must be positive")

        self.action_name = action_name
        self.feedback_interval = feedback_interval
        self.movement_duration = movement_duration
        self._clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._lock = threading.Lock()
        self._current_pose = list(start_pose or [0.0, 0.0, 0.0, 0.0])
        self._active = None

        self._action_server = ActionServer(
            self,
            PointToPoint,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info(
            f"Fake PTP action server ready on '{action_name}'."
        )

    @property
    def current_pose(self):
        """Return the most recently simulated pose."""
        with self._lock:
            return list(self._current_pose)

    def goal_callback(self, goal_request):
        """Accept only well-formed Cartesian goals with a known motion type."""
        target = list(goal_request.target_pose)
        valid = (
            goal_request.motion_type in SUPPORTED_MOTION_TYPES
            and len(target) == 4
            and all(math.isfinite(float(value)) for value in target)
        )
        if not valid:
            self.get_logger().warning("Rejecting invalid fake PTP goal.")
            return GoalResponse.REJECT

        self.get_logger().info("Fake PTP goal accepted.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept cancellation; the execute loop finalises the goal."""
        self.get_logger().info("Fake PTP cancellation accepted.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Simulate a movement and return its ``PointToPoint.Result``."""
        target = [float(value) for value in goal_handle.request.target_pose]
        result = PointToPoint.Result()

        with self._lock:
            if self._active is not None:
                goal_handle.abort()
                result.achieved_pose = list(self._current_pose)
                return result
            self._active = goal_handle
            start = list(self._current_pose)

        start_time = self._clock.now()
        pose = list(start)
        try:
            while True:
                elapsed = (
                    self._clock.now() - start_time
                ).nanoseconds / 1e9
                progress = min(max(elapsed / self.movement_duration, 0.0), 1.0)
                pose = interpolate_pose(start, target, progress)

                with self._lock:
                    self._current_pose = list(pose)

                feedback = PointToPoint.Feedback()
                feedback.current_pose = pose
                goal_handle.publish_feedback(feedback)

                if goal_handle.is_cancel_requested or progress >= 1.0:
                    break
                time.sleep(self.feedback_interval)
        finally:
            with self._lock:
                self._active = None

        result.achieved_pose = list(pose)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result


def main(args=None):
    """Run the fake PTP action server."""
    rclpy.init(args=args)
    node = FakePTPActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
