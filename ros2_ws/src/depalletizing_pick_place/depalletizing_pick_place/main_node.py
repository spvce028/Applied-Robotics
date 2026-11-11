import rclpy
import threading

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from depalletizing_interfaces.models import OperationResult
from depalletizing_pick_place.pick_place import PickPlace
from depalletizing_pick_place.robot_driver import DobotActionRobot

class DummyGripper:
    def activate_tool(self):
        return OperationResult(success=True, message="Gripper activated")
    def deactivate_tool(self):
        return OperationResult(success=True, message="Gripper deactivated")

class DepalletizingMainNode(Node):

    def __init__(self):
        super().__init__('depalletizing_main_node')
        
        # Initialize drivers
        self.robot = DobotActionRobot(self)
        self.gripper = DummyGripper()
        self.workflow = PickPlace(self.robot, self.gripper)

        self._workflow_running = False
        

        self._workflow_lock = threading.Lock()

        # Create an explicit service trigger instead of automatic startup
        self.start_service = self.create_service(
            Trigger,
            '/start_depalletizing',
            self.handle_start_service
        )

        self.get_logger().info(
            "Depalletizing node initialized in IDLE state. "
            "Call service '/start_depalletizing' to begin workflow."
        )

    def handle_start_service(self, request, response):
        """Accept a workflow request without blocking for its completion."""

        with self._workflow_lock:

            if self._workflow_running:
                response.success = False
                response.message = "A depalletizing workflow is already running."
                return response

            if self.robot.is_faulted:
                response.success = False
                response.message = "Robot driver is faulted. Recovery is required."
                return response

            if not self.robot.server_is_ready():
                response.success = False
                response.message = "Dobot Action Server is unavailable."
                return response

            self._workflow_running = True

        try:
            workflow_future = self.workflow.run_full_workflow()

            workflow_future.add_done_callback(
                self._workflow_done_callback
            )

        except Exception as exc:
            with self._workflow_lock:
                self._workflow_running = False

            self.get_logger().error(
                f"Could not start workflow: {exc}"
            )

            response.success = False
            response.message = f"Workflow could not start: {exc}"
            return response

        response.success = True
        response.message = "Depalletizing workflow accepted and started."

        self.get_logger().info(response.message)

        return response

    def _workflow_done_callback(self, future):
        """Handle completion of the asynchronous workflow."""

        try:
            result = future.result()

            if not isinstance(result, OperationResult):
                raise TypeError(
                    "Workflow must return an OperationResult."
                )

            if result.success:
                self.get_logger().info(
                    f"Workflow completed: {result.message}"
                )
            else:
                self.get_logger().error(
                    f"Workflow failed: {result.message}"
                )

        except Exception as exc:
            self.get_logger().error(
                f"Workflow completion error: {exc}"
            )

        finally:
            with self._workflow_lock:
                self._workflow_running = False

def main(args=None):
    rclpy.init(args=args)
    node = DepalletizingMainNode()

    # Use MultiThreadedExecutor to prevent deadlocks during synchronous waits / service calls
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()