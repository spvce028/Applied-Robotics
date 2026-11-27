"""Interactive M3 trials using the configured route without changing parameters."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import threading

import rclpy
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from depalletizing_pick_place.m2_lab_test import LabDriver


OBSERVATIONS = {
    'pickup': 'Was the object picked up correctly?',
    'alignment': 'Was target orientation reached BEFORE lowering?',
    'grip': 'Did the object remain securely held during transport/rotation?',
    'clearance': 'Was the path free of unwanted contact?',
    'placement': 'Was the object fully inside the container and released?',
    'autonomy': 'Did the cycle finish without manual intervention?',
}
CONFIG_NAMES = (
    [f'{prefix}_{axis}' for prefix in ('pick', 'place') for axis in 'xyzr']
    + ['approach_height', 'velocity_ratio', 'acceleration_ratio',
       'movement_timeout', 'gripper_timeout', 'keep_compressor_running',
       'gripper_service_name', 'transport_waypoints']
    + [f'workspace_{axis}_{bound}' for axis in 'xyzr' for bound in ('min', 'max')]
)


class M3LabDriver(LabDriver):
    """Reuse the M2 ROS transport while reading all required parameter types."""

    def configuration(self):
        """Read live configuration, never write it or substitute example poses."""
        reply = self._param_call(
            GetParameters, 'get_parameters', GetParameters.Request(names=CONFIG_NAMES),
        )
        fields = {
            ParameterType.PARAMETER_DOUBLE: 'double_value',
            ParameterType.PARAMETER_INTEGER: 'integer_value',
            ParameterType.PARAMETER_BOOL: 'bool_value',
            ParameterType.PARAMETER_STRING: 'string_value',
            ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value',
            ParameterType.PARAMETER_INTEGER_ARRAY: 'integer_array_value',
        }
        values = {}
        for name, value in zip(CONFIG_NAMES, reply.values):
            if name == 'transport_waypoints' and value.type == ParameterType.PARAMETER_NOT_SET:
                values[name] = []
            elif value.type in fields:
                item = getattr(value, fields[value.type])
                values[name] = list(item) if name == 'transport_waypoints' else item
            else:
                raise RuntimeError(f'Missing or unsupported parameter: {name}')
        return values

    def execute(self, service, timeout):
        """Distinguish service acceptance from completion logged by the main node."""
        self._log_event.clear()
        self._last_log = None
        accepted, message = self.call(service)
        result = {'service': service, 'accepted': accepted, 'response': message}
        if not accepted:
            return dict(result, status='rejected', software_pass=False)
        if not self._log_event.wait(timeout):
            return dict(result, status='timeout', software_pass=False)
        return dict(
            result, status='completed' if self._last_log.startswith('Operation completed')
            else 'failed', completion=self._last_log,
            software_pass=self._last_log.startswith('Operation completed'),
        )


def validated_plan(values):
    """Validate numeric bounds and require an explicit raised alignment step."""
    def pose(prefix):
        return tuple(values[f'{prefix}_{axis}'] for axis in 'xyzr')

    def finite(value):
        return type(value) in (int, float) and math.isfinite(value)

    pick, place = pose('pick'), pose('place')
    height = values['approach_height']
    if not finite(height) or height <= 0:
        raise ValueError('approach_height must be positive and finite')
    for name in ('velocity_ratio', 'acceleration_ratio'):
        if not finite(values[name]) or not 0 < values[name] <= 1:
            raise ValueError(f'{name} must be finite and in (0, 1]')
    for name in ('movement_timeout', 'gripper_timeout'):
        if not finite(values[name]) or values[name] <= 0:
            raise ValueError(f'{name} must be positive and finite')
    flat = values['transport_waypoints']
    if not isinstance(flat, list) or not flat or len(flat) % 4:
        raise ValueError('M3 needs transport_waypoints in groups of four x/y/z/r values')
    route = [tuple(flat[i:i + 4]) for i in range(0, len(flat), 4)]
    above_pick = (*pick[:2], pick[2] + height, pick[3])
    above_place = (*place[:2], place[2] + height, place[3])
    for target in [pick, place, above_pick, above_place] + route:
        for axis, value in zip('xyzr', target):
            low, high = values[f'workspace_{axis}_min'], values[f'workspace_{axis}_max']
            if not all(finite(v) for v in (value, low, high)) or not low <= value <= high:
                raise ValueError(f'Invalid or out-of-bounds {axis} in pose {target}')
    if pick[3] == place[3]:
        raise ValueError('M3 requires different pick and place orientations')
    alignment = next((
        i for i in range(1, len(route))
        if route[i - 1][:3] == route[i][:3]
        and route[i - 1][3] != route[i][3]
        and route[i][3] == place[3]
        and route[i][2] > max(above_pick[2], above_place[2])
    ), None)
    if alignment is None:
        raise ValueError('Configure a separate rotation at fixed XYZ above both approach poses')
    if any(p[3] != place[3] for p in route[alignment:]):
        raise ValueError('All waypoints after alignment must retain the place orientation')
    return [
        ('MOVJ approach pick', above_pick), ('MOVL lower', pick),
        ('gripper close', None), ('MOVL lift', above_pick),
        *[(f'MOVL transport {i + 1}', p) for i, p in enumerate(route)],
        ('MOVJ approach place', above_place), ('MOVL lower', place),
        ('gripper open', None), ('MOVL retract', above_place),
    ]


def record_summary(report):
    """Count physical and simulated outcomes separately; unknown never means pass."""
    attempted = [r for r in report['trials'] if r.get('accepted') is True]
    software = sum(r.get('software_pass') is True for r in attempted)
    physical = sum(r.get('physical_pass') is True for r in attempted)
    return {
        'attempts': len(attempted), 'software_successes': software,
        'unconfirmed_requests': sum(r.get('status') == 'requesting' for r in report['trials']),
        'physical_successes': physical if report['mode'] == 'hardware' else None,
        'physical_success_rate': (
            physical / len(attempted) if attempted and report['mode'] == 'hardware'
            else None
        ),
    }


def run_trials(node, report, save, cycles, timeout, ask=input):
    """Run individually confirmed trials and stop on uncertain or failed motion."""
    values = node.configuration()
    plan = validated_plan(values)
    report['configuration'] = values
    save()
    print('\n=== M3 oriented placement ===')
    print(f'Mode: {report["mode"]}; values are mm/degrees in base_link.')
    print('No parameters are changed. Bounds do NOT verify collisions or physical grip.')
    print('Restart the main node after changing driver/gripper settings or approach height.')
    print('Use this tool exclusively: do not send other operations concurrently.')
    for index, (name, target) in enumerate(plan, 1):
        print(f'{index:2}. {name}: {target if target is not None else ""}')
    if ask('Have you reviewed the complete path and setup? Type REVIEWED: ') != 'REVIEWED':
        report['status'] = 'not_started'
        return
    report['setup'] = {
        'object': ask('Object description (shape/dimensions): '),
        'container': ask('Container description (opening/clearance): '),
    }
    save()
    expected = 'SIMULATED' if report['mode'] == 'simulation' else 'HOMED'
    if ask(f'Confirm prior physical homing (simulation: simulated reference). Type {expected}: ') != expected:
        report['status'] = 'not_started'
        return
    ok, message = node.call('/confirm_homing')
    report['homing_confirmation'] = {'success': ok, 'message': message}
    save()
    if not ok:
        raise RuntimeError(message)
    # Opening is a separate, explicitly confirmed command, never an automatic recovery.
    if ask('Gripper may release an object. Type OPEN to prepare, otherwise stop: ') != 'OPEN':
        report['status'] = 'not_started'
        return
    report['preparation'] = {'service': '/manual/open_gripper', 'status': 'requesting'}
    save()
    prepared = node.execute('/manual/open_gripper', timeout)
    report['preparation'] = prepared
    save()
    if not prepared['software_pass']:
        raise RuntimeError('Gripper preparation failed or timed out; inspect the main node')

    for number in range(1, cycles + 1):
        if node.configuration() != values:
            raise RuntimeError('Configuration changed; restart this test after reviewing it')
        if ask(f'Object at PICK, container ready. Type START for trial {number}/{cycles} (q stops): ') != 'START':
            report['status'] = 'stopped_by_operator'
            break
        trial = {'number': number, 'status': 'requesting', 'physical_pass': None}
        report['trials'].append(trial)
        save()  # Preserve an interrupted attempt before sending a movement command.
        result = node.execute('/start_depalletizing', timeout)
        trial.update(result)
        save()
        print(result)
        if not result['software_pass']:
            report['status'] = 'stopped_after_failure'
            break
        if report['mode'] == 'hardware':
            answers = {}
            for key, question in OBSERVATIONS.items():
                answer = ask(question + ' [y/n/u=unknown]: ').strip().lower()
                answers[key] = True if answer == 'y' else False if answer == 'n' else None
                trial['observations'] = answers.copy()
                save()
            trial['physical_pass'] = all(value is True for value in answers.values())
            trial['notes'] = ask('Notes (slip/contact/fit/manual intervention): ')
            save()
            if not trial['physical_pass']:
                report['status'] = 'stopped_after_observation_failure'
                break
        else:
            print('Simulation: physical grip, clearance and container fit remain UNVERIFIED.')
    else:
        report['status'] = 'requested_trials_completed'


def main(argv=None):
    """Run the M3 operator tool and save a separate JSON session report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation', action='store_true', help='Label results as simulation only')
    parser.add_argument('--cycles', type=int, default=10)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--revision', default='not supplied', help='Tested Git commit for the report')
    parser.add_argument('--output', type=Path, help='New JSON file; existing reports are never overwritten')
    args, ros_args = parser.parse_known_args(argv)
    if args.cycles < 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('cycles must be positive; timeout must be positive and finite')
    timestamp = datetime.now(timezone.utc)
    output = args.output or Path('m3_results') / timestamp.strftime('m3_%Y%m%dT%H%M%S_%fZ.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {
        'started_utc': timestamp.isoformat(), 'mode': 'simulation' if args.simulation else 'hardware',
        'revision': args.revision, 'requested_cycles': args.cycles, 'status': 'initializing',
        'trials': [],
    }
    # Refuse an existing path before creating ROS clients or sending commands.
    with output.open('x', encoding='utf-8') as file:
        json.dump(report, file, indent=2)

    def save():
        report['summary'] = record_summary(report)
        temporary = output.with_suffix(output.suffix + '.tmp')
        temporary.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        temporary.replace(output)

    node = None
    executor = None
    spinner = None
    old_signal = None
    initialized = False
    try:
        # Keep ROS alive when Ctrl+C interrupts an input/wait, so cancel can be sent.
        rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        old_signal = signal.signal(signal.SIGINT, signal.default_int_handler)
        node = M3LabDriver()
        node.live = True
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        spinner = threading.Thread(target=executor.spin, daemon=True)
        spinner.start()
        run_trials(node, report, save, args.cycles, args.timeout)
    except (KeyboardInterrupt, EOFError):
        report['status'] = 'interrupted'
    except (RuntimeError, ValueError, KeyError) as error:
        report['status'] = 'error'
        report['error'] = str(error)
        print(f'Stopped: {error}')
    finally:
        try:
            commanded = bool(report.get('preparation') or report['trials'])
            if node is not None and commanded and report['status'] in ('interrupted', 'error', 'stopped_after_failure'):
                # No automatic reset, release or restart after an uncertain outcome.
                node._log_event.clear()
                ok, message = node.call('/cancel_depalletizing', timeout=5.0)
                report['cancel_request'] = {'accepted': ok, 'message': message}
                if ok:
                    report['cancel_request']['completion_log_received'] = node._log_event.wait(5.0)
                print(f'Cancel: {message}. Verify actual standstill before recovery.')
        finally:
            try:
                report['finished_utc'] = datetime.now(timezone.utc).isoformat()
                save()
            finally:
                if executor is not None:
                    executor.shutdown(timeout_sec=2.0)
                if spinner is not None:
                    spinner.join(timeout=2.0)
                if node is not None:
                    node.destroy_node()
                if initialized:
                    rclpy.try_shutdown()
                if old_signal is not None:
                    signal.signal(signal.SIGINT, old_signal)
            print(f'Report: {output.resolve()}\nSummary: {report["summary"]}')


if __name__ == '__main__':
    main()
