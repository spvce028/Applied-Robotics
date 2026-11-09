import unittest

from unittest.mock import Mock, patch
from dobot_msgs.srv import GripperControl
from depalletizing_pick_place.gripper_client import GripperClient
from depalletizing_interfaces.models import OperationResult

class TestGripperClient(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.service = self.node.create_client.return_value
        self.service.wait_for_service.return_value = True
        self.future = self.service.call_async.return_value
        self.future.done.return_value = True
        self.response = GripperControl.Response()
        self.response.success = True
        self.future.result.return_value = self.response
        self.gripper = GripperClient(self.node)
        spin_patch = patch("depalletizing_pick_place.gripper_client.rclpy.spin_until_future_complete")
        self.spin = spin_patch.start()
        self.addCleanup(spin_patch.stop)

    # Checks that activation sends close and returns a successful OperationResult

    def test_activate_tool_sends_close_command(self):
        response = self.gripper.activate_tool()
        self.node.create_client.assert_called_once_with(GripperControl, "/dobot_gripper_service")
        request = self.service.call_async.call_args[0][0]
        self.assertEqual(request.gripper_state, "close")
        self.assertFalse(request.keep_compressor_running)
        self.assertIsInstance(response, OperationResult)
        self.assertTrue(response.success)
        self.assertIsNone(response.error_code)

    # Checks that deactivation sends open and returns a successful OperationResult
        
    def test_deactivate_tool_sends_open_command(self):
        response = self.gripper.deactivate_tool()
        self.node.create_client.assert_called_once_with(GripperControl, "/dobot_gripper_service")
        request = self.service.call_async.call_args[0][0]
        self.assertEqual(request.gripper_state, "open")
        self.assertFalse(request.keep_compressor_running)
        self.assertIsInstance(response, OperationResult)
        self.assertTrue(response.success)
        self.assertIsNone(response.error_code)

    # Checks that no request is sent if the service is unavailable

    def test_unavailable_service_no_request(self):
        self.service.wait_for_service.return_value = False
        result = self.gripper.activate_tool()
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "GRIPPER_FAILED")
        self.service.call_async.assert_not_called()

    # Checks that a response timeout removes the pending request
    def test_response_timeout_removes_request(self):
        self.future.done.return_value = False
        result = self.gripper.activate_tool()
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "GRIPPER_FAILED")
        self.spin.assert_called_once_with(self.node, self.future, timeout_sec=5.0)
        self.service.remove_pending_request.assert_called_once_with(self.future)
        self.future.result.assert_not_called()

    # Checks that a rejected response keeps the message
    def test_rejection_keeps_message(self):
        self.response.success = False
        self.response.message = "Greiferbefehl abgelehnt"
        response = self.gripper.activate_tool()
        self.assertFalse(response.success)
        self.assertEqual(response.message, "Greiferbefehl abgelehnt")
        self.assertEqual(response.error_code, "GRIPPER_FAILED")

    # Checks that a send error returns a failure
    def test_send_error_returns_failure(self):
        self.service.call_async.side_effect = RuntimeError("Service call failed")
        result = self.gripper.activate_tool()
        self.assertFalse(result.success)
        self.assertEqual(result.error_code, "GRIPPER_FAILED")
        self.assertEqual(result.message, "Service call failed")
        self.spin.assert_not_called()