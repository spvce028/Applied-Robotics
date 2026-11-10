import rclpy
from rclpy.node import Node
from depalletizing_interfaces.models import RobotPose, OperationResult
from depalletizing_pick_place.pick_place import PickPlace
from depalletizing_pick_place.robot_driver import DobotActionRobot


class DepalletizingMainNode(Node):

    def __init__(self):
        super().__init__('depalletizing_main_node')
        
        self._workflow_started = False
        
        # 1. Initialize drivers
        self.robot_driver = DobotActionRobot(self)
        self.robot_driver.wait_for_server(timeout_sec=5.0)
        self.gripper_driver = DummyGripper() 

        # 2. Initialize PickPlace logic
        self.pick_place_logic = PickPlace(
            robot=self.robot_driver, 
            gripper=self.gripper_driver
        )

        # 3. Trigger workflow asynchronously using a timer
        self.create_timer(1.0, self.execute_workflow_async)

    def execute_workflow_async(self):
        if self._workflow_started:
            return
        self._workflow_started = True
        
        target = RobotPose(x=200.0, y=0.0, z=50.0, r=0.0, frame_id="base_link")
        self.get_logger().info("Starting asynchronous pick operation...")
        
        # Call pick_object. If your PickPlace class triggers robot.move_to, 
        # make sure it returns the future chain properly.
        movement_future = self.pick_place_logic.pick_object(target)
        
        # Attach a non-blocking callback to handle the final result when complete
        movement_future.add_done_callback(self.handle_pick_result)

    def handle_pick_result(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Pick operation completed successfully asynchronously!")
            else:
                self.get_logger().error(f"Pick operation failed: {result.message}")
        except Exception as exc:
            self.get_logger().error(f"Workflow execution exception: {exc}")


class DummyGripper:
    def activate_tool(self):
        return OperationResult(success=True, message="Gripper activated")
    def deactivate_tool(self):
        return OperationResult(success=True, message="Gripper deactivated")


def main(args=None):
    rclpy.init(args=args)
    node = DepalletizingMainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()