import rclpy
from rclpy.node import Node
from depalletizing_interfaces.models import RobotPose
from depalletizing_pick_place.pick_place import PickPlace
from depalletizing_pick_place.robot_driver import DobotActionRobot


class DepalletizingMainNode(Node):

    def __init__(self):
        super().__init__('depalletizing_main_node')
        
        # 1. Initialize your Dobot Action Client robot driver
        self.robot_driver = DobotActionRobot(self)
        
        # 2. Initialize your gripper driver (replace with your actual gripper implementation)
        self.gripper_driver = DummyGripper() 

        # 3. Inject them into your existing PickPlace class
        self.pick_place_logic = PickPlace(
            robot=self.robot_driver, 
            gripper=self.gripper_driver
        )

        # 4. Run an example sequence
        self.execute_workflow()

    def execute_workflow(self):
        target = RobotPose(x=200.0, y=0.0, z=50.0, r=0.0, frame_id="base_link")
        self.get_logger().info("Starting pick operation...")
        
        result = self.pick_place_logic.pick_object(target)
        if result.success:
            self.get_logger().info("Pick operation completed successfully!")
        else:
            self.get_logger().error(f"Pick operation failed: {result.message}")


# Dummy placeholder for your gripper if you haven't implemented it yet
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