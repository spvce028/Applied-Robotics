# Gripper client for depalletizing pick and place operations
# sends open and close commands to a service and returns the result

import rclpy
from dobot_msgs.srv import GripperControl
from rclpy.node import Node
from depalletizing_interfaces.models import OperationResult

class GripperClient:
    def __init__(self, node: Node, service_name: str = "/dobot_gripper_service"):
        self.node = node
        self.client = node.create_client(GripperControl, service_name)

    def send_request(self, state, keep_compressor_running):
        if state not in ("open", "close"):
            raise ValueError("Invalid Gripper state. Must be 'open' or 'close'")
        
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise ConnectionError("Gripper Service unavailable")

        request = GripperControl.Request()
        request.gripper_state = state
        request.keep_compressor_running = keep_compressor_running

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)

        if not future.done():
            self.client.remove_pending_request(future)
            raise TimeoutError("No Gripper Response")
        
        return future.result()

    def operate(self, state: str) -> OperationResult:
        try:
            response = self.send_request(state, keep_compressor_running = False)
        except (ConnectionError, TimeoutError, RuntimeError) as error:
            return OperationResult(success=False, message=str(error), error_code="GRIPPER_FAILED")
        return OperationResult(success = response.success, message = response.message, error_code=None if response.success else "GRIPPER_FAILED")
            

    def activate_tool(self) -> OperationResult:
        return self.operate("close")

    def deactivate_tool(self) -> OperationResult:
        return self.operate("open")

    