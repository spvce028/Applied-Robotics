"""Fake gripper service for simulation.

Same name and type as the real driver. Parameter ``mode``:
``success``, ``fail`` or ``no_response`` (answers only after 30 s).
Switch at runtime: ``ros2 param set /fake_gripper_service mode fail``
"""

import time
import math
import threading

import rclpy
from dobot_msgs.srv import GripperControl
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FakeGripperService(Node):

    def __init__(self, *, service_name='/dobot_gripper_service', response_delay=0.2,
                 no_response_delay=30.0):
        """Create a configurable service with observable simulated state."""
        super().__init__("fake_gripper_service")
        self.declare_parameter("mode", "success")
        self.declare_parameter('service_name', service_name)
        self.declare_parameter('response_delay', response_delay)
        self.declare_parameter('no_response_delay', no_response_delay)
        for name in ('response_delay', 'no_response_delay'):
            value = self.get_parameter(name).value
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and non-negative')
        self._state_lock = threading.Lock()
        self._requests = []
        self._is_closed = False
        self.create_service(
            GripperControl, self.get_parameter('service_name').value, self.handle_request,
            callback_group=ReentrantCallbackGroup(),
        )

    @property
    def requests(self):
        """Return a snapshot of received (state, compressor) commands."""
        with self._state_lock:
            return list(self._requests)

    @property
    def is_closed(self):
        """Return simulated jaw state, not physical grasp verification."""
        with self._state_lock:
            return self._is_closed

    def handle_request(self, request, response):
        """Reply according to the configured mode and record valid commands."""
        mode = self.get_parameter("mode").value
        with self._state_lock:
            self._requests.append((request.gripper_state, request.keep_compressor_running))
        self.get_logger().info(f"{request.gripper_state} (mode={mode})")

        delay = self.get_parameter(
            'no_response_delay' if mode == 'no_response' else 'response_delay'
        ).value
        time.sleep(delay)

        response.success = mode in ('success', 'no_response') and request.gripper_state in ('open', 'close')
        if response.success:
            with self._state_lock:
                self._is_closed = request.gripper_state == 'close'
        response.message = "fake " + ("failure" if mode == "fail" else "ok")
        return response


def main():
    rclpy.init()
    node = FakeGripperService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
