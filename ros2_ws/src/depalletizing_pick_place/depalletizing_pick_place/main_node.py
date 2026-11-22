import rclpy
import threading
import math
from array import array

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.exceptions import ParameterUninitializedException
from rclpy.task import Future
from std_srvs.srv import Trigger

from depalletizing_interfaces.models import OperationResult, RobotPose
from depalletizing_interfaces.workflow import PickAndPlaceWorkflow
from depalletizing_interfaces.waypoints import WaypointExecutor
from depalletizing_pick_place.pick_place import MOVL, PickPlace
from depalletizing_pick_place.robot_driver import DobotActionRobot, WorkspaceLimits
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

        # ROS parameters cannot contain nested lists. Each four numbers form
        # one pose: [x, y, z, r, x, y, z, r, ...]. Empty means direct pick/place.
        # Dynamic typing permits an empty YAML list; values are checked below.
        self.declare_parameter(
            'transport_waypoints', [],
            descriptor=ParameterDescriptor(dynamic_typing=True),
        )

        # Optional workspace limits (conservative defaults)
        self.declare_parameter('workspace_x_min', -320.0)
        self.declare_parameter('workspace_x_max', 320.0)
        self.declare_parameter('workspace_y_min', -320.0)
        self.declare_parameter('workspace_y_max', 320.0)
        self.declare_parameter('workspace_z_min', -100.0)
        self.declare_parameter('workspace_z_max', 300.0)
        self.declare_parameter('workspace_r_min', -360.0)
        self.declare_parameter('workspace_r_max', 360.0)

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


        try:
            workspace_limits = WorkspaceLimits(
                x=(
                    self.get_parameter('workspace_x_min').value,
                    self.get_parameter('workspace_x_max').value,
                ),
                y=(
                    self.get_parameter('workspace_y_min').value,
                    self.get_parameter('workspace_y_max').value,
                ),
                z=(
                    self.get_parameter('workspace_z_min').value,
                    self.get_parameter('workspace_z_max').value,
                ),
                r=(
                    self.get_parameter('workspace_r_min').value,
                    self.get_parameter('workspace_r_max').value,
                ),
            )
        except ValueError as exc:
            message = f'Invalid workspace limits: {exc}'
            self.get_logger().fatal(message)
            raise RuntimeError(message) from exc

        self._workspace_limits = workspace_limits
        # Reject malformed/out-of-bounds transport before creating any drivers.
        self._read_transport_waypoints()

        # Initialize drivers
        self.robot = DobotActionRobot(
            self,
            velocity_ratio=self.get_parameter('velocity_ratio').value,
            acceleration_ratio=self.get_parameter('acceleration_ratio').value,
            movement_timeout=self.get_parameter('movement_timeout').value,
            workspace_limits=workspace_limits,
        )

        self.gripper = GripperClient(
            self,
            service_name=self.get_parameter('gripper_service_name').value,
            timeout_sec=self.get_parameter('gripper_timeout').value,
            keep_compressor_running=(
                self.get_parameter('keep_compressor_running').value
            ),
        )

        self._cancel_requested = threading.Event()

        self.pick_place = PickPlace(
            self.robot,
            self.gripper,
            approach_height=self.get_parameter('approach_height').value,
            is_cancelled=self._cancel_requested.is_set,
        )

        self.waypoint_executor = WaypointExecutor(
            lambda pose: self.robot.move_to(pose, motion_type=MOVL),
            is_cancelled=self._cancel_requested.is_set,
        )

        self.workflow = PickAndPlaceWorkflow(
            self.pick_place.pick_object,
            self.pick_place.place_object,
            is_cancelled=self._cancel_requested.is_set,
            move_through_waypoints=self.waypoint_executor.move_through_waypoints,
        )

        self._workflow_running = False

        # Block new starts while cancellation is handed to the robot driver.
        self._cancel_dispatching = False

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

        self.cancel_service = self.create_service(
            Trigger,
            '/cancel_depalletizing',
            self.handle_cancel_service,
        )

        self.reset_fault_service = self.create_service(
            Trigger,
            '/reset_fault',
            self.handle_reset_fault,
        )

        self.manual_pick_service = self.create_service(
            Trigger,
            '/manual/pick',
            self.handle_manual_pick,
        )

        self.manual_place_service = self.create_service(
            Trigger,
            '/manual/place',
            self.handle_manual_place,
        )

        self.manual_open_gripper_service = self.create_service(
            Trigger,
            '/manual/open_gripper',
            self.handle_manual_open_gripper,
        )

        self.manual_close_gripper_service = self.create_service(
            Trigger,
            '/manual/close_gripper',
            self.handle_manual_close_gripper,
        )

        self.get_logger().info(
            "Depalletizing node initialized in IDLE state. "
            "Call service '/start_depalletizing' to begin workflow."
        )

    def handle_confirm_homing(self, request, response):
        """Record manual homing confirmation without moving the robot."""
        with self._workflow_lock:
            if (
                self._workflow_running
                or self._cancel_dispatching
                or self.robot.is_busy
            ):
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

    def handle_cancel_service(self, request, response):
        """Request cancellation without waiting for movement completion."""
        with self._workflow_lock:
            if self._cancel_dispatching:
                response.success = False
                response.message = 'A cancellation request is being dispatched.'
                return response

            # Cancellation must also work when homing or reachability checks fail.
            if not self._workflow_running and not self.robot.is_busy:
                response.success = False
                response.message = 'No active operation to cancel.'
                return response

            # Prevent subsequent pick/place steps, including after a gripper reply.
            self._cancel_requested.set()
            self._cancel_dispatching = True

        try:
            # Driver callbacks can finish the workflow and acquire our lock.
            # Dispatch outside it; _cancel_dispatching blocks a new start meanwhile.
            self.robot.cancel_current_goal()
            response.success = True
            response.message = (
                'Cancellation requested. Robot standstill is not yet confirmed.'
            )
        except Exception as error:
            # Keep the event set even if the driver cannot send the request.
            response.success = False
            response.message = f'Failed to request robot cancellation: {error}'
            self.get_logger().error(response.message)
        finally:
            with self._workflow_lock:
                self._cancel_dispatching = False

        # Completion callbacks, not this acknowledgement, release the workflow.
        return response

    def handle_reset_fault(self, request, response):
        """Reset an idle driver without overriding its movement lock."""
        with self._workflow_lock:
            # Recovery must not overlap a workflow, movement or cancel dispatch.
            if (
                self._workflow_running
                or self._cancel_dispatching
                or self.robot.is_busy
            ):
                response.success = False
                response.message = (
                    'Cannot reset fault while an operation may still be active.'
                )
                return response

            if not self.robot.is_faulted:
                response.success = True
                response.message = 'Robot driver is not faulted. No reset needed.'
                return response

            # Recovery does not establish a valid reference position. Require
            # fresh manual homing confirmation before any subsequent movement.
            self._homing_confirmed = False
            try:
                # Never force-release an uncertain movement in a service reset.
                reset = self.robot.reset_fault(force=False)
            except Exception as error:
                response.success = False
                response.message = f'Failed to reset robot fault: {error}'
                self.get_logger().error(response.message)
                return response

            response.success = bool(reset) and not self.robot.is_faulted
            if response.success:
                response.message = (
                    'Robot driver fault reset. Verify the robot reference '
                    'and confirm manual homing before starting again.'
                )
            else:
                response.message = 'Robot driver fault reset was refused.'

        # This service neither starts a workflow nor clears a pending cancel event.
        return response

    def _check_preconditions(self) -> str | None:
        """Return a rejection reason or None; caller must hold the lock."""
        if self._workflow_running or self._cancel_dispatching:
            return 'An operation is already running.'

        if not self._homing_confirmed:
            return (
                'Homing has not been confirmed. '
                'Perform homing manually and confirm it before starting.'
            )

        if self.robot.is_faulted:
            return 'Robot driver is faulted. Recovery is required.'

        if self.robot.is_busy:
            return 'Robot driver is busy.'

        if not self.robot.server_is_ready():
            return 'Dobot Action Server is unavailable.'

        if not self.gripper.service_is_ready():
            return 'Gripper service is unavailable.'

        return None

    def handle_manual_pick(self, request, response):
        """Request a pick at the configured source pose."""
        return self._start_manual_operation(
            lambda: self.pick_place.pick_object(self._read_pose('pick')),
            response,
        )

    def handle_manual_place(self, request, response):
        """Request placement at the configured target pose."""
        return self._start_manual_operation(
            lambda: self.pick_place.place_object(self._read_pose('place')),
            response,
        )

    def handle_manual_open_gripper(self, request, response):
        """Request opening through the same guards as a robot movement."""
        return self._start_manual_operation(
            lambda: self._operate_manual_gripper(closed=False),
            response,
        )

    def handle_manual_close_gripper(self, request, response):
        """Request closing through the same guards as a robot movement."""
        return self._start_manual_operation(
            lambda: self._operate_manual_gripper(closed=True),
            response,
        )

    def _operate_manual_gripper(self, *, closed: bool) -> Future:
        """Keep the pick/place holding state consistent with manual commands."""
        operation = (
            self.gripper.activate_tool if closed
            else self.gripper.deactivate_tool
        )
        result = operation()
        if isinstance(result, OperationResult):
            command_future = Future()
            command_future.set_result(result)
        elif isinstance(result, Future):
            command_future = result
        else:
            raise TypeError('Gripper must return Future or OperationResult.')

        completed = Future()

        def on_done(future):
            try:
                result = future.result()
                if not isinstance(result, OperationResult):
                    raise TypeError('Gripper Future must contain OperationResult.')

                if result.success:
                    # Closed means "may hold an object", not verified grasping.
                    # Record the real command outcome even after a cancel request.
                    self.pick_place.holding_object = closed

                # Update holding state BEFORE releasing the shared execution slot.
                completed.set_result(result)
            except Exception as error:
                # The common completion handler logs errors and releases the slot.
                completed.set_exception(error)

        command_future.add_done_callback(on_done)
        return completed

    def _start_manual_operation(self, operation, response):
        """Accept a manual operation using the shared execution lock."""
        with self._workflow_lock:
            rejection_reason = self._check_preconditions()

            if rejection_reason is not None:
                response.success = False
                response.message = rejection_reason
                return response

            self._cancel_requested.clear()
            self._workflow_running = True

        try:
            # Start outside the lock: completion callbacks may acquire it.
            result = operation()

            if isinstance(result, OperationResult):
                future = Future()
                future.set_result(result)
            elif isinstance(result, Future):
                future = result
            else:
                raise TypeError(
                    'Manual operation must return Future or OperationResult.'
                )

            future.add_done_callback(self._operation_done_callback)
        except Exception as error:
            with self._workflow_lock:
                self._workflow_running = False

            response.success = False
            response.message = f'Could not start manual operation: {error}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.message = 'Manual operation accepted.'
        return response

    # Helper-Method for shared pose Parameters
    def _read_pose(self, prefix: str) -> RobotPose:
        """Read a configured pick or place pose."""
        return RobotPose(
            x=self.get_parameter(f'{prefix}_x').value,
            y=self.get_parameter(f'{prefix}_y').value,
            z=self.get_parameter(f'{prefix}_z').value,
            r=self.get_parameter(f'{prefix}_r').value,
        )

    def _read_transport_waypoints(self) -> tuple[RobotPose, ...]:
        """Parse and validate every configured transport pose before picking."""
        values = self.get_parameter('transport_waypoints').value
        if not isinstance(values, (list, tuple, array)):
            raise RuntimeError('transport_waypoints must be a numeric list.')
        if len(values) % 4:
            raise RuntimeError(
                'transport_waypoints must contain groups of four: x, y, z, r.'
            )
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in values
        ):
            raise RuntimeError('transport_waypoints must contain finite numbers.')

        poses = []
        for index in range(0, len(values), 4):
            pose = RobotPose(*(float(v) for v in values[index:index + 4]))
            violations = self._workspace_limits.violations(pose)
            if violations:
                raise RuntimeError(
                    f'transport_waypoints pose {index // 4 + 1}: '
                    + '; '.join(violations)
                )
            poses.append(pose)
        return tuple(poses)

    def handle_start_service(self, request, response):
        """Accept a workflow request without blocking for its completion."""

        with self._workflow_lock:
            rejection_reason = self._check_preconditions()

            if rejection_reason is not None:
                response.success = False
                response.message = rejection_reason
                return response

            self._cancel_requested.clear()
            self._workflow_running = True

        try:

            # Read pick and place positions from ROS 2 parameters.
            pick_pose = self._read_pose('pick')
            place_pose = self._read_pose('place')
            # Snapshot and validate the whole route before the first movement.
            transport_poses = self._read_transport_waypoints()

            workflow_future = self.workflow.execute_pick_and_place(

                pick_pose,
                place_pose,
                transport_waypoints=transport_poses,
            )

            workflow_future.add_done_callback(
                self._operation_done_callback
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

    def _operation_done_callback(self, future):
        """Handle completion of a workflow or manual operation."""
        try:
            result = future.result()

            if not isinstance(result, OperationResult):
                raise TypeError(
                    'Operation must return an OperationResult.'
                )

            if result.success:
                self.get_logger().info(
                    f'Operation completed: {result.message}'
                )
            else:
                self.get_logger().error(
                    f'Operation failed: {result.message}'
                )

        except Exception as exc:
            self.get_logger().error(
                f'Operation completion error: {exc}'
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
