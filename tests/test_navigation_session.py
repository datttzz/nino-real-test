"""Pose/frame/map fixtures and isolated lifecycle services; no robot commands."""
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import time
from unittest.mock import patch

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('navigation_session', ROOT/'scripts/navigation_session.py')
helper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helper)


def message():
    return dict(header=dict(frame_id='map', stamp=dict(sec=100, nanosec=0)),
                pose=dict(pose=dict(position=dict(x=1., y=-2., z=0.),
                    orientation=dict(x=0., y=0., z=math.sin(.3), w=math.cos(.3)))))


def map_file(tmp_path):
    (tmp_path/'room.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    p = tmp_path/'room.yaml'
    p.write_text('image: room.pgm\nresolution: 0.05\norigin: [0,0,0]\n')
    return p


def test_saved_rviz_pose_roundtrip_and_map_change_rejected(tmp_path):
    m = map_file(tmp_path)
    data = helper.pose_record(message(), helper.map_identity(m), 100.2)
    out = tmp_path/'initial_pose.json'
    out.write_text(json.dumps(data))
    assert list(map(float, helper.read_pose(out, m).split(','))) == pytest.approx([1., -2., .6])
    (tmp_path/'room.pgm').write_bytes(b'changed')
    with pytest.raises(ValueError, match='Map YAML'):
        helper.read_pose(out, m)


@pytest.mark.parametrize('kind', ['frame', 'old', 'future', 'quaternion', 'nan'])
def test_bad_initial_pose_not_saved(kind):
    m = message()
    if kind == 'frame': m['header']['frame_id'] = 'odom'
    if kind == 'old': m['header']['stamp']['sec'] = 90
    if kind == 'future': m['header']['stamp']['sec'] = 102
    if kind == 'quaternion': m['pose']['pose']['orientation']['w'] = 0
    if kind == 'nan': m['pose']['pose']['position']['x'] = float('nan')
    with pytest.raises(ValueError):
        helper.pose_record(m, {}, 100.2)


def test_active_or_transitioning_nodes_are_not_reconfigured():
    assert helper.next_transition(1) == 1
    assert helper.next_transition(2) == 3
    assert all(helper.next_transition(state) is None for state in [None, 0, 3, 4, 10, 13])


@pytest.mark.parametrize('group', ['nav_only', 'pi', 'sac80k'])
@pytest.mark.parametrize('initial_pose', ['', '0.25,-0.12,1.57'])
def test_launch_arguments_with_and_without_saved_pose(group, initial_pose):
    pytest.importorskip('ros2launch')
    from ros2launch.api.api import parse_launch_arguments
    source = (ROOT/'scripts/run_logged_real.sh').read_text()
    # Chạy đúng khối dựng argv của wrapper, không chạy recorder/launch/motor.
    chunk = 'nav_args=(' + source.split('nav_args=(', 1)[1].split('\nset +e', 1)[0]
    selected = 'nav_args' if group == 'nav_only' else 'args'
    env = dict(os.environ, group=group, initial_pose=initial_pose,
               controller={'nav_only': '', 'pi': 'pi', 'sac80k': 'pi_sac'}[group],
               map_file='/test/room with spaces.yaml', hardware_file='/test/hardware.json',
               session_dir='/test/run')
    run = subprocess.run(['bash', '-c', chunk + '\nprintf "%s\\0" "${' + selected + '[@]}"'],
                         env=env, capture_output=True, check=True)
    argv = [s.decode() for s in run.stdout.split(b'\0') if s]
    parsed = dict(parse_launch_arguments(argv))
    assert parsed['map'] == '/test/room with spaces.yaml'
    if initial_pose:
        assert parsed['initial_pose'] == initial_pose
    else:
        assert 'initial_pose' not in parsed
    if group != 'nav_only':
        assert parsed['controller'] == env['controller']
    assert ('checkpoint' in parsed) == (group == 'sac80k')


def test_actual_ready_cli_with_isolated_lifecycle_servers(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from lifecycle_msgs.srv import GetState, ChangeState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    env = dict(os.environ, ROS_DOMAIN_ID='218', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    env.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)
    env.pop('FASTDDS_DEFAULT_PROFILES_FILE', None)
    with patch.dict(os.environ, env, clear=True):
        rclpy.init(args=[])
        node = rclpy.create_node('test_localization_services_only')
        states = {'map_server': 2, 'amcl': 1}  # the observed incident: inactive / unconfigured
        calls = []
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(OccupancyGrid, '/map', qos)

        def get(name, request, response):
            response.current_state.id = states[name]
            return response

        def change(name, request, response):
            transition = request.transition.id
            calls.append((name, transition))
            response.success = ((states[name], transition) in [(1, 1), (2, 3)])
            if response.success:
                states[name] = 2 if transition == 1 else 3
                if name == 'map_server' and states[name] == 3:
                    m = OccupancyGrid(); m.header.frame_id = 'map'
                    m.info.width = 2; m.info.height = 2; m.data = [0, 0, 0, 0]
                    pub.publish(m)
            return response

        services = []
        for name in states:
            services += [node.create_service(GetState, '/'+name+'/get_state',
                lambda req, res, name=name: get(name, req, res)),
                node.create_service(ChangeState, '/'+name+'/change_state',
                lambda req, res, name=name: change(name, req, res))]
        out = tmp_path/'ready.json'
        proc = subprocess.Popen(['/usr/bin/python3', str(ROOT/'scripts/navigation_session.py'),
            'ready', '--grace', '0', '--timeout', '18', '--result', str(out)],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            deadline = time.monotonic()+23
            while proc.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            if proc.poll() is None:
                proc.kill()
            stdout, _ = proc.communicate(timeout=3)
            assert proc.returncode == 0, stdout
            result = json.loads(out.read_text())
            assert result['passed'] and result['states'] == {'map_server': 3, 'amcl': 3}
            assert calls == [('map_server', 3), ('amcl', 1), ('amcl', 3)]
        finally:
            if proc.poll() is None: proc.kill(); proc.wait()
            node.destroy_node(); rclpy.shutdown()
