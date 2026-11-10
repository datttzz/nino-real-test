#!/usr/bin/env python3
"""Lưu pose RViz và kiểm tra lifecycle định vị; không phát lệnh motor/goal."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import yaml


def map_identity(filename):
    path = Path(filename).expanduser().resolve(strict=True)
    image = Path(yaml.safe_load(path.read_text())['image']).expanduser()
    if not image.is_absolute():
        image = path.parent / image
    return dict(yaml_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                image_sha256=hashlib.sha256(image.read_bytes()).hexdigest())


def pose_record(message, identity, now_ros):
    """Chỉ nhận pose map mới từ RViz, kiểm tra quaternion trước khi lấy yaw."""
    if message['header']['frame_id'] != 'map':
        raise ValueError('Initial pose phải ở frame map')
    stamp = message['header']['stamp']
    stamp = stamp['sec'] + stamp['nanosec'] * 1e-9
    if not math.isfinite(stamp) or not -.05 <= now_ros - stamp <= 5:
        raise ValueError('Initial pose có timestamp cũ/tương lai')
    pose = message['pose']['pose']
    x, y = pose['position']['x'], pose['position']['y']
    q = [pose['orientation'][k] for k in ('x', 'y', 'z', 'w')]
    if not all(math.isfinite(v) for v in [x, y, *q]):
        raise ValueError('Pose không hữu hạn')
    norm = math.sqrt(sum(v*v for v in q))
    if abs(norm-1) > .01:
        raise ValueError('Quaternion không chuẩn hóa')
    qx, qy, qz, qw = [v/norm for v in q]
    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
    return dict(format_version=1, frame_id='map', x=x, y=y, yaw=yaw,
                source='/initialpose: operator RViz estimate, not ground truth',
                source_stamp_ros_s=stamp, map=identity,
                original_message=message)


def read_pose(filename, map_file):
    data = json.loads(Path(filename).read_text())
    if data.get('format_version') != 1 or data.get('frame_id') != 'map':
        raise ValueError('Sai định dạng/frame của file pose')
    if data.get('map') != map_identity(map_file):
        raise ValueError('Map YAML/ảnh đã đổi; phải lưu pose cho map này')
    values = [data[k] for k in ('x', 'y', 'yaw')]
    if not all(type(v) in (float, int) and math.isfinite(v) for v in values):
        raise ValueError('Pose không hữu hạn')
    return ','.join(format(v, '.17g') for v in values)


def next_transition(state):
    # Trạng thái chuyển tiếp phải chờ, không gửi trùng transition đang chạy.
    return {1: 1, 2: 3}.get(state)  # unconfigured -> configure; inactive -> activate


def run_ros(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from lifecycle_msgs.srv import GetState, ChangeState
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from rosidl_runtime_py.convert import message_to_ordereddict

    rclpy.init(args=[])
    node = Node('navigation_session_' + args.command.replace('-', '_'))
    deadline = time.monotonic() + args.timeout
    result = dict(passed=False, command=args.command, transitions=[])

    def call(client, request):
        remaining = deadline-time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=min(1., remaining)):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future,
                                       timeout_sec=max(0., min(3., deadline-time.monotonic())))
        if future.done():
            return future.result()
        client.remove_pending_request(future)
        return None

    try:
        if args.command == 'save-pose':
            out = Path(args.output).expanduser()
            if out.exists():
                raise ValueError('File pose đã có; chọn tên mới, không ghi đè pose cũ')
            identity = map_identity(args.map)
            received = []
            sub = node.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                           lambda msg: received.append(msg), 10)
            print('Đang chờ: dùng 2D Pose Estimate trong RViz; xe phải ở vạch đầu.', flush=True)
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if not received:
                raise TimeoutError('Chưa nhận pose RViz; chưa lưu file')
            data = pose_record(message_to_ordereddict(received[-1]), identity,
                               node.get_clock().now().nanoseconds*1e-9)
            out.parent.mkdir(parents=True, exist_ok=True)
            with out.open('x') as stream:
                stream.write(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
            print('Đã lưu:', out.resolve(), 'pose=', read_pose(out, args.map), flush=True)
            result.update(passed=True, pose_file=str(out.resolve()))
            node.destroy_subscription(sub)
        else:
            maps = []
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            sub = node.create_subscription(OccupancyGrid, '/map', lambda m: maps.append(m), qos)
            clients = {name: (node.create_client(GetState, '/'+name+'/get_state'),
                              node.create_client(ChangeState, '/'+name+'/change_state'))
                       for name in ('map_server', 'amcl')}
            grace_end = time.monotonic()+args.grace
            states = {}
            attempts = {name: 0 for name in clients}
            while time.monotonic() < deadline:
                for name, (get, change) in clients.items():
                    response = call(get, GetState.Request())
                    state = response.current_state.id if response else None
                    states[name] = state
                    # Cho lifecycle manager tự khởi động trước; chỉ phục hồi khi cần.
                    if time.monotonic() < grace_end or state == 3:
                        continue
                    # Map phải active trước khi sửa trạng thái AMCL.
                    if name == 'amcl' and states.get('map_server') != 3:
                        continue
                    transition = next_transition(state)
                    if transition is not None:
                        if attempts[name] >= 3:
                            raise RuntimeError('Quá số lần phục hồi lifecycle: '+name)
                        attempts[name] += 1
                        request = ChangeState.Request()
                        request.transition.id = transition
                        reply = call(change, request)
                        result['transitions'].append(dict(node=name, from_state=state,
                            transition=transition, response=None if reply is None else reply.success))
                        print('Lifecycle:', name, 'transition=', transition,
                              'response=', None if reply is None else reply.success, flush=True)
                        # Luôn đọc lại state ở vòng kế tiếp, kể cả mất response.
                rclpy.spin_once(node, timeout_sec=.2)
                if all(states.get(name) == 3 for name in clients) and maps:
                    m = maps[-1]
                    if m.header.frame_id != 'map' or not m.info.width or not m.info.height:
                        raise ValueError('Map sai frame hoặc rỗng')
                    result.update(passed=True, states=states, map_width=m.info.width,
                                  map_height=m.info.height)
                    print('READY: map_server active; amcl active; /map đã nhận. '
                          'Chưa xác nhận pose/laser hoặc bật motor.', flush=True)
                    break
            if not result['passed']:
                raise TimeoutError('Định vị chưa sẵn sàng: '+str(states))
            node.destroy_subscription(sub)
    except Exception as exc:
        result['error'] = type(exc).__name__+': '+str(exc)
        print(result['error'], file=sys.stderr, flush=True)
    finally:
        if args.command == 'ready' and args.result:
            out = Path(args.result)
            with out.open('x') as stream:
                stream.write(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    ready = commands.add_parser('ready', help='Phục hồi map_server/AMCL, kiểm tra /map')
    ready.add_argument('--timeout', type=float, default=90.)
    ready.add_argument('--grace', type=float, default=20.)
    ready.add_argument('--result')
    save = commands.add_parser('save-pose', help='Chờ một pose mới từ RViz')
    save.add_argument('--map', required=True)
    save.add_argument('--output', required=True)
    save.add_argument('--timeout', type=float, default=180.)
    read = commands.add_parser('read-pose', help='Đọc pose đã lưu, xác minh map; không ROS')
    read.add_argument('--map', required=True)
    read.add_argument('--file', required=True)
    args = parser.parse_args()
    if args.command == 'read-pose':
        try:
            print(read_pose(args.file, args.map))
            return 0
        except (OSError, ValueError, KeyError) as exc:
            parser.error(str(exc))
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout phải dương, hữu hạn')
    if args.command == 'ready' and (not math.isfinite(args.grace) or args.grace < 0):
        parser.error('--grace phải không âm, hữu hạn')
    return run_ros(args)


if __name__ == '__main__':
    raise SystemExit(main())
