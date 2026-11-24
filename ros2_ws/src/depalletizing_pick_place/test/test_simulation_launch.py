"""Smoke-test the installed simulation launch in an isolated ROS domain."""

import os
import signal
import subprocess
import time
import uuid

import pytest

pytest.importorskip('rclpy.node')
pytest.importorskip('dobot_msgs.action')

import rclpy
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl
from rclpy.action import ActionClient
from std_srvs.srv import Trigger


@pytest.mark.parametrize('simulation', [True, False], ids=['simulation', 'hardware'])
def test_launch_has_no_autostart(tmp_path, simulation):
    """Select fake servers only in simulation; neither mode starts motion."""
    domain = 100 + uuid.uuid4().int % 70
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    log_path = tmp_path / 'simulation-launch.log'
    process = None
    caller = None
    initialized = False
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(
                ['ros2', 'launch', 'depalletizing_pick_place',
                 'depalletizing.launch.py', f'simulation:={str(simulation).lower()}',
                 'use_rviz:=false'],
                env=environment, stdout=log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            rclpy.init(domain_id=domain)
            initialized = True
            caller = rclpy.create_node('simulation_launch_smoke_test')

            start = caller.create_client(Trigger, '/start_depalletizing')
            homing = caller.create_client(Trigger, '/confirm_homing')
            cancel = caller.create_client(Trigger, '/cancel_depalletizing')
            reset = caller.create_client(Trigger, '/reset_fault')

            for client in (start, homing, cancel, reset):
                assert client.wait_for_service(
                    timeout_sec=15.0
                ), log_path.read_text()

            action = ActionClient(caller, PointToPoint, '/PTP_action')
            assert action.wait_for_server(timeout_sec=3.0) == simulation, log_path.read_text()

            gripper = caller.create_client(GripperControl, '/dobot_gripper_service')
            assert gripper.wait_for_service(timeout_sec=1.0) == simulation, log_path.read_text()

            reply = start.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(caller, reply, timeout_sec=5.0)

            assert reply.done(), log_path.read_text()
            assert not reply.result().success
            assert 'Homing' in reply.result().message

            time.sleep(0.1)
            logs = log_path.read_text()

            assert 'Sending Dobot movement goal' not in logs, logs
            assert 'Fake PTP goal accepted' not in logs, logs
            assert 'close (mode=' not in logs, logs
            assert 'open (mode=' not in logs, logs
            assert process.poll() is None, logs

            action.destroy()
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
        if caller is not None:
            caller.destroy_node()
        if initialized:
            rclpy.try_shutdown()
