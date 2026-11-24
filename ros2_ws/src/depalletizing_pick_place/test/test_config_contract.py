"""Check the shipped P6 configuration against P5's parameter declarations."""

import ast
from pathlib import Path

import pytest

yaml = pytest.importorskip('yaml')
PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize(
    'config_name',
    ['depalletizing.yaml', 'simulation.yaml'],
)
def test_yaml_matches_main_node_parameters(config_name):
    """Keep both shipped configurations aligned with main-node parameters."""
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / config_name).read_text(encoding='utf-8')
    )
    assert set(config) == {'depalletizing_main_node'}
    values = config['depalletizing_main_node']['ros__parameters']
    tree = ast.parse(
        (PACKAGE_ROOT / 'depalletizing_pick_place' / 'main_node.py').read_text(
            encoding='utf-8-sig'
        )
    )
    declarations = {}
    for call in ast.walk(tree):
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Attribute)
            and call.func.attr == 'declare_parameter'
            and isinstance(call.func.value, ast.Name)
            and call.func.value.id == 'self'
        ):
            declarations[ast.literal_eval(call.args[0])] = call.args[1]

    assert not (set(values) - set(declarations)), 'YAML contains unknown parameters'
    types = {'DOUBLE': float, 'STRING': str, 'BOOL': bool}
    for name, declaration in declarations.items():
        if isinstance(declaration, ast.Attribute) and declaration.attr in types:
            assert name in values, f'Missing required YAML parameter: {name}'
            assert type(values[name]) is types[declaration.attr], name


def test_launch_uses_shipped_configuration():
    """Verify that launch references both configurations and the node name."""
    launch_file = PACKAGE_ROOT / 'launch' / 'depalletizing.launch.py'
    tree = ast.parse(launch_file.read_text(encoding='utf-8-sig'))
    constants = {
        n.value for n in ast.walk(tree)
        if isinstance(n, ast.Constant) and isinstance(n.value, str)
    }
    assert {
        'config',
        'simulation',
        'depalletizing.yaml',
        'simulation.yaml',
        'depalletizing_main_node',
    } <= constants
