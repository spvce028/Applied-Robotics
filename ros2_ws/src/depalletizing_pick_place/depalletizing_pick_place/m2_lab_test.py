"""Interactive M2 lab tool: teach poses, step-by-step test and full demo.

  ros2 run depalletizing_pick_place m2_lab_test measure  # teach pick/place -> YAML
  ros2 run depalletizing_pick_place m2_lab_test run      # step through the M2 test
  ros2 run depalletizing_pick_place m2_lab_test demo     # full cycle incl. transport

`measure` needs the dobot bringup (it reads /dobot_pose_raw). `run` and `demo`
need the depalletizing node, backed either by the real robot or by
fake_ptp_action_server + fake_gripper_service (simulation).
Ctrl+C during a step sends /cancel_depalletizing.

`measure` writes to ./src/depalletizing_pick_place/config/depalletizing.yaml
when started from the workspace root, otherwise to the installed config
(resolved through the --symlink-install link). Override with --config PATH.
"""

import argparse
import os
import re
import shutil
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rcl_interfaces.msg import Log, Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

NODE_NAME = 'depalletizing_main_node'
FIRST_TEST_SPEED = 0.2
STEP_TIMEOUT = 90.0
DONE_PREFIXES = (
    'Operation completed', 'Operation failed', 'Operation completion error')


class LabDriver(Node):
    """ROS helper: pose reading, Trigger calls and node parameter access."""

    def __init__(self):
        super().__init__('m2_lab_test')
        self._pose = None
        self._pose_event = threading.Event()
        self._log_event = threading.Event()
        self._last_log = None
        self.live = False
        self.create_subscription(Float64MultiArray, '/dobot_pose_raw', self._on_pose, 10)
        self.create_subscription(Log, '/rosout', self._on_log, 50)

    def _on_pose(self, msg):
        self._pose = list(msg.data)
        self._pose_event.set()

    def _on_log(self, msg):
        if msg.name != NODE_NAME:
            return
        if self.live:
            print(f'    [node] {msg.msg}')
        if msg.msg.startswith(DONE_PREFIXES):
            self._last_log = msg.msg
            self._log_event.set()

    def read_pose_mm(self, timeout=5.0):
        self._pose_event.clear()
        if not self._pose_event.wait(timeout):
            raise RuntimeError('No /dobot_pose_raw message. Is the dobot bringup running?')
        x, y, z, r = self._pose[:4]
        # dobot_pose_raw publishes metres; our config uses mm.
        return [round(x * 1000, 1), round(y * 1000, 1), round(z * 1000, 1), round(r, 1)]

    def call(self, name, timeout=10.0):
        client = self.create_client(Trigger, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return False, f'Service {name} not available'
            future = client.call_async(Trigger.Request())
            end = time.time() + timeout
            while not future.done() and time.time() < end:
                time.sleep(0.05)
            if not future.done():
                return False, f'No response from {name}'
            res = future.result()
            return res.success, res.message
        finally:
            self.destroy_client(client)

    def _param_call(self, srv_type, suffix, request, timeout=5.0):
        client = self.create_client(srv_type, f'/{NODE_NAME}/{suffix}')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'{NODE_NAME} is not running (terminal 2?)')
            future = client.call_async(request)
            end = time.time() + timeout
            while not future.done() and time.time() < end:
                time.sleep(0.05)
            if not future.done():
                raise RuntimeError(f'No reply from /{NODE_NAME}/{suffix}')
            return future.result()
        finally:
            self.destroy_client(client)

    def get_params(self, names):
        reply = self._param_call(
            GetParameters, 'get_parameters', GetParameters.Request(names=names))
        values = {}
        for name, value in zip(names, reply.values):
            is_double = value.type == ParameterType.PARAMETER_DOUBLE
            values[name] = value.double_value if is_double else None
        return values

    def set_waypoints(self, flat):
        param = Parameter(name='transport_waypoints', value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            double_array_value=[float(v) for v in flat]))
        reply = self._param_call(
            SetParameters, 'set_parameters', SetParameters.Request(parameters=[param]))
        result = reply.results[0]
        if not result.successful:
            raise RuntimeError(f'transport_waypoints rejected: {result.reason}')

    def call_and_wait(self, name):
        self._log_event.clear()
        self._last_log = None
        ok, msg = self.call(name)
        print(f'  -> {name}: {"OK" if ok else "REJECTED"} ({msg})')
        if not ok:
            return False
        print('  ... waiting for the operation to finish (Ctrl+C = cancel)')
        if not self._log_event.wait(STEP_TIMEOUT):
            print(f'  !! no completion after {STEP_TIMEOUT:.0f}s - check terminal 2')
            return False
        print(f'  <- {self._last_log}')
        return self._last_log.startswith('Operation completed')


def ask(prompt):
    """Ask for Enter/skip/quit; quit exits the tool."""
    answer = input(f'{prompt} [Enter = go, s = skip, q = quit] ').strip().lower()
    if answer == 'q':
        raise SystemExit(0)
    return answer != 's'


def default_config_path():
    """Prefer the source YAML so taught poses can be committed."""
    source = Path.cwd() / 'src/depalletizing_pick_place/config/depalletizing.yaml'
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory('depalletizing_pick_place'))
    return Path(os.path.realpath(share / 'config' / 'depalletizing.yaml'))


def update_yaml_text(text, values):
    """Replace `key: value` lines in place, keeping comments and layout."""
    for key, value in values.items():
        text, count = re.subn(
            rf'^(\s*{key}:\s*)[^#\n]*?([ \t]*(#.*)?)$', rf'\g<1>{value}\g<2>',
            text, flags=re.M)
        if count != 1:
            raise RuntimeError(f'Key {key} not found exactly once in the config')
    return text


def write_yaml(path, values):
    """Write taught values to the config, keeping a backup in ~/.ros."""
    text = update_yaml_text(path.read_text(), values)
    backup = Path.home() / '.ros' / 'depalletizing.yaml.bak'
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy(path, backup)
    path.write_text(text)
    print(f'Written to {path}\n(backup of the previous file: {backup})')
    if '/install/' in str(path):
        print('!! This is an installed copy; a rebuild overwrites it.')


def measure(node, config):
    print('\n=== Teach poses ===')
    print(f'Config: {config}')
    print('Hold the unlock button on the forearm and move the arm by hand.')
    print('Measure at GRASP height (the node approaches from 50 mm above).\n')
    poses = {}
    for name in ('pick', 'place'):
        input(f'Move the gripper to the {name.upper()} pose, release the button, '
              'then press Enter ')
        poses[name] = node.read_pose_mm()
        x, y, z, r = poses[name]
        print(f'  {name}: x={x} y={y} z={z} r={r}  (mm/deg)')

    values = {}
    for name, (x, y, z, r) in poses.items():
        values.update({f'{name}_x': x, f'{name}_y': y, f'{name}_z': z, f'{name}_r': r})
    values['velocity_ratio'] = FIRST_TEST_SPEED
    values['acceleration_ratio'] = FIRST_TEST_SPEED

    print(f'\nAlso sets velocity_ratio/acceleration_ratio = {FIRST_TEST_SPEED} '
          'for the first runs.')
    if input('Write these values to depalletizing.yaml? [y/N] ').strip().lower() == 'y':
        write_yaml(config, values)
        print('\nRestart the node (terminal 2):')
        print('  ros2 launch depalletizing_pick_place depalletizing.launch.py')
        print('Then (terminal 3):')
        print('  ros2 run depalletizing_pick_place m2_lab_test demo')


def run(node, config=None):
    """Step through gripper, manual pick/place and the full workflow."""
    print('\n=== M2 hardware test ===')
    print('Keep the emergency stop within reach. Ctrl+C during a step cancels the movement.\n')
    steps = [
        ('Confirm homing (only after homing has finished)', '/confirm_homing', False),
        ('Open the gripper', '/manual/open_gripper', True),
        ('Close the gripper', '/manual/close_gripper', True),
        ('Open the gripper', '/manual/open_gripper', True),
        ('Put the object on the PICK pose. Pick it up', '/manual/pick', True),
        ('Place the object on the PLACE pose', '/manual/place', True),
        ('Object back on PICK. Full workflow', '/start_depalletizing', True),
    ]
    results = []
    for title, service, wait in steps:
        print(f'\n--- {title}')
        if not ask('Run?'):
            results.append((title, 'skipped'))
            continue
        try:
            if wait:
                ok = node.call_and_wait(service)
            else:
                ok, msg = node.call(service)
                print(f'  -> {service}: {"OK" if ok else "REJECTED"} ({msg})')
        except KeyboardInterrupt:
            ok, msg = node.call('/cancel_depalletizing')
            print(f'\n  !! cancel requested: {msg}')
            ok = False
        results.append((title, 'PASS' if ok else 'FAIL'))
        if not ok:
            if node.call('/reset_fault', timeout=3.0)[0]:
                print('  (fault reset)')
            if not ask('Step failed. Continue anyway?'):
                break

    print('\n=== Summary ===')
    for title, status in results:
        print(f'  {status:8} {title}')


PLACEHOLDER = {'pick': (200.0, 0.0, 50.0, 0.0), 'place': (250.0, 100.0, 50.0, 0.0)}
TRANSPORT_CLEARANCE = 30.0  # mm above the higher approach pose


def transport_via(pick, place, approach, z_max):
    """Midpoint between pick and place, raised above both approach poses."""
    via_z = min(max(pick[2], place[2]) + approach + TRANSPORT_CLEARANCE, z_max)
    return (round((pick[0] + place[0]) / 2, 1), round((pick[1] + place[1]) / 2, 1),
            round(via_z, 1), round((pick[3] + place[3]) / 2, 1))


def demo(node, config=None):
    """Full M2 cycle in one go: open -> pick -> transport -> place, with live log."""
    keys = [f'{p}_{a}' for p in ('pick', 'place') for a in 'xyzr']
    keys += ['approach_height', 'workspace_z_max', 'velocity_ratio']
    params = node.get_params(keys)
    pick = tuple(params[f'pick_{a}'] for a in 'xyzr')
    place = tuple(params[f'place_{a}'] for a in 'xyzr')
    approach = params['approach_height']

    via = transport_via(pick, place, approach, params['workspace_z_max'])

    def fmt(pose):
        return f'x={pose[0]:7.1f}  y={pose[1]:7.1f}  z={pose[2]:7.1f}  r={pose[3]:6.1f}'

    def above(pose):
        return (pose[0], pose[1], pose[2] + approach, pose[3])

    print('\n=== M2 demo: pick & place from known pose to predefined pose ===')
    print(f'velocity_ratio = {params["velocity_ratio"]}\n')
    plan = [
        ('gripper', 'open'),
        ('MOVJ', f'above pick     {fmt(above(pick))}'),
        ('MOVL', f'down to pick   {fmt(pick)}'),
        ('gripper', 'close (grasp)'),
        ('MOVL', f'lift           {fmt(above(pick))}'),
        ('MOVL', f'transport via  {fmt(via)}'),
        ('MOVJ', f'above place    {fmt(above(place))}'),
        ('MOVL', f'down to place  {fmt(place)}'),
        ('gripper', 'open (release)'),
        ('MOVL', f'lift           {fmt(above(place))}'),
    ]
    for i, (kind, text) in enumerate(plan, 1):
        print(f'  {i:2}. {kind:7} {text}')

    if pick == PLACEHOLDER['pick'] or place == PLACEHOLDER['place']:
        print('\n!! pick/place are still the PLACEHOLDER values from the repo.')
        print('!! Run "m2_lab_test measure" and restart the node first.')
        if input('Continue anyway? [y/N] ').strip().lower() != 'y':
            return

    print('\nObject on the PICK pose, workspace clear, emergency stop in reach.')
    if not ask('Start demo?'):
        return

    node.set_waypoints(via)
    ok, msg = node.call('/confirm_homing')
    print(f'  -> /confirm_homing: {"OK" if ok else "REJECTED"} ({msg})')
    if not ok:
        return

    node.live = True
    started = time.time()
    try:
        for title, service in (('Open gripper', '/manual/open_gripper'),
                               ('Pick -> transport -> place', '/start_depalletizing')):
            print(f'\n--- {title}')
            try:
                ok = node.call_and_wait(service)
            except KeyboardInterrupt:
                print(f'\n  !! cancel requested: {node.call("/cancel_depalletizing")[1]}')
                ok = False
            if not ok:
                print('\n=== DEMO FAILED ===')
                return
    finally:
        node.live = False
        try:
            node.set_waypoints([])
        except RuntimeError as error:
            print(f'(could not reset transport_waypoints: {error})')

    print(f'\n=== DEMO PASSED in {time.time() - started:.1f}s ===')


def main(argv=None):
    """Parse the mode and run it with a background executor."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mode', choices=('measure', 'run', 'demo'))
    parser.add_argument('--config', type=Path, help='YAML written by measure')
    args, ros_args = parser.parse_known_args(argv)
    config = args.config or (default_config_path() if args.mode == 'measure' else None)
    rclpy.init(args=ros_args)
    node = LabDriver()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        {'measure': measure, 'run': run, 'demo': demo}[args.mode](node, config)
    except (KeyboardInterrupt, EOFError):
        print('\nAborted.')
    except RuntimeError as error:
        print(f'\nError: {error}')
    finally:
        executor.shutdown()
        spinner.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
