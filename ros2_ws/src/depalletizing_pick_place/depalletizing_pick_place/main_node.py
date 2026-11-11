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
        self.create_timer(1.0, self.execute_workflow)

    def execute_workflow(self):
        if self._workflow_started:
            return
        self._workflow_started = True
        
        self.get_logger().info("Starting full pick-and-place workflow...")

        # Example poses for pick and place
        pick_pose = RobotPose(x=200.0, y=0.0, z=50.0, r=0.0, frame_id="base_link")
        place_pose = RobotPose(x=150.0, y=150.0, z=50.0, r=0.0, frame_id="base_link")

        # Run pick workflow
        pick_result = self.pick_place_logic.pick_object(pick_pose)
        if not pick_result.success:
            self.get_logger().error(f"Pick failed: {pick_result.message}")
            return

        self.get_logger().info("Pick successful! Executing place operation...")

        # Run place workflow
        place_result = self.pick_place_logic.place_object(place_pose)
        if not place_result.success:
            self.get_logger().error(f"Place failed: {place_result.message}")
            return

        self.get_logger().info("Full pick-and-place workflow completed successfully!")


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