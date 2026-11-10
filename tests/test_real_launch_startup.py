"""Inspect launch actions offline; never run LaunchService or hardware nodes."""
import importlib.util
import json
from pathlib import Path
import sys
from unittest.mock import patch

import pytest

pytest.importorskip('launch_ros')
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/nino_real_rl'))
spec = importlib.util.spec_from_file_location(
    'tested_real_launch', ROOT / 'src/nino_real_rl/launch/real_test.launch.py')
launch_file = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_file)


def context(mode, hardware='', checkpoint=''):
    ctx = LaunchContext()
    ctx.launch_configurations.update(dict(controller=mode, hardware_file=hardware,
        checkpoint=checkpoint, map='/unused/offline_test_map.yaml',
        robot_config=str(ROOT / 'src/pi5_slam_bringup/config/robot.yaml'),
        rviz='false', initial_pose='', stm32_port='', lidar_port='', log_dir=''))
    return ctx


@pytest.mark.parametrize('hardware', ['', str(ROOT / 'hardware.example.json'), '/missing/hardware.json'])
def test_invalid_hardware_prevents_any_node_action(hardware):
    with patch.object(launch_file, 'Node') as node, patch.object(
            launch_file, 'get_package_share_directory') as package:
        with pytest.raises(RuntimeError, match='Controller startup blocked'):
            launch_file.setup(context('pi', hardware))
        node.assert_not_called()
        package.assert_not_called()


def test_shadow_still_does_not_require_hardware():
    with patch.object(launch_file, 'get_package_share_directory',
                      side_effect=lambda name: str(ROOT / 'src' / name)), patch.object(
            launch_file, 'OnProcessExit', wraps=launch_file.OnProcessExit) as on_exit:
        actions = launch_file.setup(context('shadow'))
    assert on_exit.call_args.kwargs['target_action'] is actions[-1]
    assert len(on_exit.call_args.kwargs['on_exit']) == 1


def test_pi_controller_exit_is_bound_to_launch_shutdown(tmp_path):
    # Test-only assertions; not proof of any actual robot verification.
    data = json.loads((ROOT / 'hardware.example.json').read_text())
    for key in ('firmware_torque_verified', 'wheel_torque_nm_verified', 'watchdog_verified',
                'feedback_signs_verified', 'accept_feedback_rate_difference'):
        data[key] = True
    data.update(left_command_sign=1, right_command_sign=1,
                verified_wheel_torque_limit_nm=2, verified_watchdog_seconds=.5)
    file = tmp_path / 'test_only_hardware.json'
    file.write_text(json.dumps(data))
    with patch.object(launch_file, 'get_package_share_directory',
                      side_effect=lambda name: str(ROOT / 'src' / name)), patch.object(
            launch_file, 'OnProcessExit', wraps=launch_file.OnProcessExit) as on_exit:
        actions = launch_file.setup(context('pi', str(file)))
    assert on_exit.call_args.kwargs['target_action'] is actions[-1]
