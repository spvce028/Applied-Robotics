import unittest

from unittest.mock import Mock

from rclpy.task import Future

from dobot_msgs.srv import GripperControl
from depalletizing_pick_place.gripper_client import GripperClient

class TestGripperClient(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.service = self.node.create_client.return_value
        self.service.service_is_ready.return_value = True
        self.service_future = Future()
        self.service.call_async.return_value = self.service_future
        self.gripper = GripperClient(self.node)

    def respond(self, success=True, message=""):
        service_future = self.service.call_async.return_value
        service_future.set_result(GripperControl.Response(success=success, message=message))

    # Checks that activation sends close
    def test_activate_tool_sends_close_command(self):
        future = self.gripper.activate_tool()
        self.assertFalse(future.done())

        request = self.service.call_async.call_args[0][0]
        self.assertEqual(request.gripper_state, "close")
        self.assertFalse(request.keep_compressor_running)
        self.assertEqual(self.node.create_timer.call_args[0][0], 5.0)

        self.respond(success=True)

        self.assertTrue(future.result().success)
        self.node.destroy_timer.assert_called_once()

    # Checks that deactivation sends open    
    def test_deactivate_tool_sends_open_command(self):
        future = self.gripper.deactivate_tool()
        self.assertEqual(self.service.call_async.call_args[0][0].gripper_state, "open")

        self.respond(success=True)

        self.assertTrue(future.result().success)

    # Checks that parameters are taken from constructor
    def test_constructor(self):
        gripper = GripperClient(
            self.node,
            service_name="/custom",
            timeout_sec=2.0,
            keep_compressor_running=True
        )

        gripper.activate_tool()

        self.node.create_client.assert_called_with(GripperControl, "/custom")
        self.assertTrue(self.service.call_async.call_args[0][0].keep_compressor_running)
        self.assertEqual(self.node.create_timer.call_args[0][0], 2.0)

    # Checks that no request is sent if the service is unavailable
    def test_unavailable_service_no_request(self):
        self.service.service_is_ready.return_value = False

        future = self.gripper.activate_tool()

        self.assertEqual(future.result().error_code, "GRIPPER_FAILED")
        self.service.call_async.assert_not_called()

    # Checks that a rejected response returns GRIPPER_FAILED and keeps the message
    def test_rejected_response(self):
        future = self.gripper.activate_tool()

        self.respond(success=False, message="Invalid request")

        self.assertEqual(future.result().error_code, "GRIPPER_FAILED")
        self.assertIn("Invalid request", future.result().message)

    # Checks Exception calls GRIPPER_FAILED
    def test_service_exception(self):
        future = self.gripper.activate_tool()

        self.service.call_async.return_value.set_exception(RuntimeError("Service call failed"))

        self.assertEqual(future.result().error_code, "GRIPPER_FAILED")

    # Checks that timeout releases client & destroys timer
    def test_timeout_releases_client(self):
        future = self.gripper.activate_tool()
        timer = self.node.create_timer.return_value
        timeout_callback = self.node.create_timer.call_args[0][1]
        timeout_callback()

        self.assertEqual(future.result().error_code, "TIMEOUT")
        self.node.destroy_timer.assert_called_with(timer)

        self.service.call_async.return_value = Future()
        next_future = self.gripper.deactivate_tool()
        self.assertEqual(self.service.call_async.call_count, 2)
        self.assertFalse(next_future.done())

    # Checks that late response of a timed-out command is ignored
    def test_late_response_ignored(self):
        old_service_future = self.service.call_async.return_value
        old = self.gripper.activate_tool()
        self.node.create_timer.call_args[0][1]()

        self.service.call_async.return_value = Future()
        new = self.gripper.deactivate_tool() 

        old_service_future.set_result(GripperControl.Response(success=True, message="late"))
        self.assertEqual(old.result().error_code, "TIMEOUT")
        self.assertFalse(new.done())

    # Checks rejection of second command while first is pending
    def test_reject_second_command_while_pending(self):
        future1 = self.gripper.activate_tool()
        future2 = self.gripper.deactivate_tool()

        self.assertEqual(future2.result().error_code, "GRIPPER_FAILED")
        self.assertFalse(future1.done())
        self.service.call_async.assert_called_once()

if __name__ == "__main__":
    unittest.main()

