import rclpy
from rclpy.node import Node
from depalletizing_interfaces.models import RobotPose, OperationResult
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
        self.gripper = DummyGripper(self)
        self.workflow = PickPlaceWorkflow(self.robot, self.gripper, self)

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
        """Operator-triggered service callback to safely start the workflow."""
        self.get_logger().info("Received start trigger. Checking server availability...")

        if not self.robot.wait_for_server(timeout_sec=3.0):
            response.success = False
            response.message = "Aborted: Dobot Action Server is offline."
            return response

        self.get_logger().info("Starting pick-and-place workflow sequence...")
        
        # Execute workflow synchronously in a safe manner
        success = self.workflow.run_full_workflow()

        if success:
            response.success = True
            response.message = "Depalletizing workflow completed successfully."
        else:
            response.success = False
            response.message = "Depalletizing workflow failed or was aborted."
            
        return response


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