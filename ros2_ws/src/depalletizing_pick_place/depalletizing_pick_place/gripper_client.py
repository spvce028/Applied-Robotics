# Asynchronous Gripper client for depalletizing pick and place operations

import threading

from rclpy.node import Node
from rclpy.task import Future

from depalletizing_interfaces.errors import ERROR_GRIPPER_FAILED, ERROR_TIMEOUT
from depalletizing_interfaces.models import OperationResult

from dobot_msgs.srv import GripperControl

def failed(message: str) -> OperationResult:
    return OperationResult(
        success=False, message=message, error_code=ERROR_GRIPPER_FAILED
    )

# Sends open and close commands to the gripper
class GripperClient:
    def __init__(
            self,
            node: Node,
            service_name: str = "/dobot_gripper_service",
            timeout_sec: float = 5.0,
            keep_compressor_running: bool = False,
            ):
        self.node = node
        self.timeout_sec = timeout_sec
        self.keep_compressor_running = keep_compressor_running

        self._client = node.create_client(GripperControl, service_name)
        self._lock = threading.RLock()
        self._pending = None
        self._timer = None
    
    def activate_tool(self) -> Future:
        return self._send("close")

    def deactivate_tool(self) -> Future:
        return self._send("open")

    def service_is_ready(self) -> bool:
        return self._client.service_is_ready()

    def _send(self, state: str) -> Future:
        future = Future()

        with self._lock:
            if self._pending is not None:
                future.set_result(failed("Another gripper command is pending"))
                return future

            if not self.service_is_ready():
                future.set_result(failed("Gripper service is unavailable"))
                return future

            self._pending = future
            self._timer = self.node.create_timer(self.timeout_sec, lambda: self._on_timeout(future))

            request = GripperControl.Request(
                gripper_state=state,
                keep_compressor_running=self.keep_compressor_running
            )
            try:
                self._client.call_async(request).add_done_callback(lambda f: self._on_response(f, future))
            except Exception as error:
                self._finish(future, failed(f"Failed to send gripper command: {error}"))
                
        return future

    def _on_response(self, service_future, future: Future):
        try:
            response = service_future.result()
        except Exception as error:
            self._finish(future, failed(f"Failed to get gripper response: {error}"))
            return

        if response.success:
            self._finish(future, OperationResult(success=True, message=response.message))
        else:
            self._finish(future, failed(f"Gripper command failed: {response.message}"))

    def _on_timeout(self, future: Future):
        self.node.get_logger().error("Gripper command timed out")
        self._finish(future, OperationResult(success=False, message="No Gripper response received before timeout", error_code=ERROR_TIMEOUT))

    def _finish(self, future: Future, result: OperationResult):
        with self._lock:
            if self._pending is not future:
                return
            if self._timer is not None:
                self.node.destroy_timer(self._timer)
                self._timer = None

            self._pending = None
            future.set_result(result)