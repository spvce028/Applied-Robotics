"""Fake gripper service for simulation.

Same name and type as the real driver. Parameter ``mode``:
``success``, ``fail`` or ``no_response`` (answers only after 30 s).
Switch at runtime: ``ros2 param set /fake_gripper_service mode fail``
"""

import time

import rclpy
from dobot_msgs.srv import GripperControl
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FakeGripperService(Node):

    def __init__(self):
        super().__init__("fake_gripper_service")
        self.declare_parameter("mode", "success")
        self.create_service(
            GripperControl, "/dobot_gripper_service", self.handle_request,
            callback_group=ReentrantCallbackGroup(),
        )

    def handle_request(self, request, response):
        mode = self.get_parameter("mode").value
        self.get_logger().info(f"{request.gripper_state} (mode={mode})")

        time.sleep(30.0 if mode == "no_response" else 0.2)

        response.success = mode != "fail"
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
