import rclpy
import threading
import math

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ParameterUninitializedException
from std_srvs.srv import Trigger

from depalletizing_interfaces.models import OperationResult, RobotPose
from depalletizing_interfaces.workflow import PickAndPlaceWorkflow
from depalletizing_pick_place.pick_place import PickPlace
from depalletizing_pick_place.robot_driver import DobotActionRobot
from depalletizing_pick_place.gripper_client import GripperClient


class DepalletizingMainNode(Node):

    def __init__(self, *, parameter_overrides=None):
        super().__init__(
            'depalletizing_main_node',
            parameter_overrides=parameter_overrides,
        )

        # Required pick and place positions, without default values
        self.declare_parameter('pick_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pick_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pick_z', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('pick_r', rclpy.Parameter.Type.DOUBLE)

        self.declare_parameter('place_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('place_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('place_z', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('place_r', rclpy.Parameter.Type.DOUBLE)

        # Required motion parameters
        self.declare_parameter('velocity_ratio', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('acceleration_ratio', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('movement_timeout', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('approach_height', rclpy.Parameter.Type.DOUBLE)

        # Required gripper parameters
        self.declare_parameter('gripper_service_name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('gripper_timeout', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('keep_compressor_running', rclpy.Parameter.Type.BOOL)

        # Only Position Params
        required_pose_parameters = (
            'pick_x', 'pick_y', 'pick_z', 'pick_r',
            'place_x', 'place_y', 'place_z', 'place_r',
        )

        # ALL Params
        required_parameters = required_pose_parameters + (
            'velocity_ratio',
            'acceleration_ratio',
            'movement_timeout',
            'approach_height',
            'gripper_service_name',
            'gripper_timeout',
            'keep_compressor_running',
        )

        # Checking if Params are missing
        missing_parameters = []

        # Loop checks all Params
        for name in required_parameters:
            try:
                self.get_parameter(name)
            except ParameterUninitializedException:
                missing_parameters.append(name)

        if missing_parameters:
            message = (
                'Missing required parameters: '
                + ', '.join(missing_parameters)
            )
            self.get_logger().fatal(message)
            raise RuntimeError(message)

        # Checking if Params are invalid for Position Params
        invalid_parameters = []

        for name in required_pose_parameters:
            value = self.get_parameter(name).value

            if not math.isfinite(value):
                invalid_parameters.append(name)

        if invalid_parameters:
            message = (
                'Parameters must be finite numbers: '
                + ', '.join(invalid_parameters)
            )
            self.get_logger().fatal(message)
            raise RuntimeError(message)


        parameter_errors = []

        # Checking if Params are invalid for the other Params (if they are in scope, etc.)
        for name in ('velocity_ratio', 'acceleration_ratio'):
            value = self.get_parameter(name).value

            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                parameter_errors.append(f'{name} must be within (0.0, 1.0]')

        for name in ('movement_timeout', 'gripper_timeout', 'approach_height'):
            value = self.get_parameter(name).value

            if not math.isfinite(value) or value <= 0.0:
                parameter_errors.append(f'{name} must be finite and greater than zero')

        service_name = self.get_parameter('gripper_service_name').value

        if not service_name.strip():
            parameter_errors.append('gripper_service_name must not be empty')

        if parameter_errors:
            message = 'Invalid configuration: ' + '; '.join(parameter_errors)
            self.get_logger().fatal(message)
            raise RuntimeError(message)


        # Initialize drivers
        self.robot = DobotActionRobot(
            self,
            velocity_ratio=self.get_parameter('velocity_ratio').value,
            acceleration_ratio=self.get_parameter('acceleration_ratio').value,
            movement_timeout=self.get_parameter('movement_timeout').value,
        )

        self.gripper = GripperClient(
            self,
            service_name=self.get_parameter('gripper_service_name').value,
            timeout_sec=self.get_parameter('gripper_timeout').value,
            keep_compressor_running=(
                self.get_parameter('keep_compressor_running').value
            ),
        )

        self.pick_place = PickPlace(
            self.robot,
            self.gripper,
            approach_height=self.get_parameter('approach_height').value,
        )

        self.workflow = PickAndPlaceWorkflow(
            self.pick_place.pick_object, self.pick_place.place_object,
        )

        self._workflow_running = False

        self._homing_confirmed = False

        self._workflow_lock = threading.Lock()

        # Create an explicit service trigger instead of automatic startup
        self.start_service = self.create_service(
            Trigger,
            '/start_depalletizing',
            self.handle_start_service
        )

        self.confirm_homing_service = self.create_service(
            Trigger,
            '/confirm_homing',
            self.handle_confirm_homing,
        )

        self.get_logger().info(
            "Depalletizing node initialized in IDLE state. "
            "Call service '/start_depalletizing' to begin workflow."
        )

    def handle_confirm_homing(self, request, response):
        """Record manual homing confirmation without moving the robot."""
        with self._workflow_lock:
            if self._workflow_running or self.robot.is_busy:
                response.success = False
                response.message = (
                    'Cannot confirm homing while an operation is active.'
                )
                return response

            if self.robot.is_faulted:
                response.success = False
                response.message = (
                    'Cannot confirm homing while the robot driver is faulted.'
                )
                return response

            self._homing_confirmed = True

        response.success = True
        response.message = 'Manual homing confirmed. No movement was commanded.'
        self.get_logger().info(response.message)
        return response

    def handle_start_service(self, request, response):
        """Accept a workflow request without blocking for its completion."""

        with self._workflow_lock:

            if self._workflow_running:
                response.success = False
                response.message = "A depalletizing workflow is already running."
                return response

            if not self._homing_confirmed:
                response.success = False
                response.message = (
                    'Homing has not been confirmed. '
                    'Perform homing manually and confirm it before starting.'
                )
                return response

            if self.robot.is_faulted:
                response.success = False
                response.message = "Robot driver is faulted. Recovery is required."
                return response

            if not self.robot.server_is_ready():
                response.success = False
                response.message = "Dobot Action Server is unavailable."
                return response

            if not self.gripper.service_is_ready():
                response.success = False
                response.message = "Gripper service is unavailable."
                return response

            self._workflow_running = True

        try:

            # Read pick and place positions from ROS 2 parameters.
            pick_pose = RobotPose(
                x=self.get_parameter('pick_x').value,
                y=self.get_parameter('pick_y').value,
                z=self.get_parameter('pick_z').value,
                r=self.get_parameter('pick_r').value,
            )

            place_pose = RobotPose(
                x=self.get_parameter('place_x').value,
                y=self.get_parameter('place_y').value,
                z=self.get_parameter('place_z').value,
                r=self.get_parameter('place_r').value,
            )

            workflow_future = self.workflow.execute_pick_and_place(

                pick_pose,
                place_pose,
            )

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
