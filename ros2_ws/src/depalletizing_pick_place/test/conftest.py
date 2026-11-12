"""Provide rclpy.task.Future when the tests run outside a ROS install."""

import sys
import types


def _install_future_stub():
    rclpy = types.ModuleType("rclpy")
    task = types.ModuleType("rclpy.task")

    class Future:
        """Minimal stand-in for rclpy.task.Future."""

        def __init__(self):
            self._done = False
            self._result = None
            self._exception = None
            self._callbacks = []

        def done(self):
            return self._done

        def set_result(self, result):
            if self._done:
                raise RuntimeError("Future is already done")
            self._result = result
            self._done = True
            callbacks = self._callbacks
            self._callbacks = []
            for callback in callbacks:
                callback(self)

        def set_exception(self, exception):
            if self._done:
                raise RuntimeError("Future is already done")
            self._exception = exception
            self._done = True
            callbacks = self._callbacks
            self._callbacks = []
            for callback in callbacks:
                callback(self)

        def result(self):
            if not self._done:
                raise RuntimeError("Future is not done")
            if self._exception is not None:
                raise self._exception
            return self._result

        def add_done_callback(self, callback):
            if self._done:
                callback(self)
                return
            self._callbacks.append(callback)

    task.Future = Future
    rclpy.task = task
    sys.modules["rclpy"] = rclpy
    sys.modules["rclpy.task"] = task


try:
    from rclpy.task import Future  # noqa: F401
except ModuleNotFoundError:
    _install_future_stub()
