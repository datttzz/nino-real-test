#!/usr/bin/env python3
"""Thử từng bánh trên giá kê; không chạy Nav2/SAC hoặc sửa hardware JSON."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray, UInt8
import yaml


def wheel_command(wheel, omega, radius, separation):
    """Rad/s tại bánh -> Twist, bánh còn lại yêu cầu đứng yên."""
    left, right = (omega, 0.) if wheel == 0 else (0., omega)
    return radius*(left+right)/2, radius*(right-left)/separation


def travel_check(delta, other_delta, target):
    if not all(math.isfinite(v) for v in (delta, other_delta, target)):
        raise RuntimeError('Góc quay không hữu hạn')
    if delta < -.15:
        raise RuntimeError('Encoder quay ngược dấu yêu cầu; dừng để kiểm tra chiều')
    if abs(other_delta) > .15:
        raise RuntimeError('Bánh không được chọn cũng quay; kiểm tra mapping/chế độ tay')
    if delta > target+.15:
        raise RuntimeError('Vượt ngưỡng góc quay; dừng')
    return delta >= target


class Probe(Node):
    def __init__(self, out):
        super().__init__('visible_wheel_stand_test')
        self.samples = {}
        self.phase = 'discovery'
        self.raw = (out/'samples.jsonl').open('x')
        self.subs = [self.create_subscription(cls, topic,
                     lambda msg, t=topic: self.receive(t, msg), qos_profile_sensor_data)
                     for topic, cls in [('/motor_fb', JointState), ('/imu/data', Imu),
                                        ('/motor_status', UInt8), ('/imu/debug', Float64MultiArray)]]
        self.vel = self.create_publisher(Twist, '/cmd_vel', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.enable = self.create_publisher(Bool, '/motor_enable', 1)

    def receive(self, topic, msg):
        data = message_to_ordereddict(msg)
        self.samples[topic] = (time.monotonic(), data)
        self.raw.write(json.dumps(dict(phase=self.phase, topic=topic,
            received_monotonic_s=self.samples[topic][0], message=data))+'\n')

    def spin(self):
        rclpy.spin_once(self, timeout_sec=.005)

    def check(self, enabled=False):
        now = time.monotonic()
        for topic, timeout in [('/motor_fb', .15), ('/imu/data', .25), ('/motor_status', .35)]:
            if topic not in self.samples or now-self.samples[topic][0] > timeout:
                debug = self.samples.get('/imu/debug') if topic == '/imu/data' else None
                raise RuntimeError(f'Missing/stale {topic}; IMU diagnostic={debug}')
            if topic != '/motor_status':
                stamp = self.samples[topic][1]['header']['stamp']
                age = self.get_clock().now().nanoseconds*1e-9-stamp['sec']-stamp['nanosec']*1e-9
                if not -.05 <= age <= .3:
                    raise RuntimeError('Timestamp cũ/tương lai: '+topic)
        status = self.samples['/motor_status'][1]['data']
        if status & 3 != 3 or status & 12 or (enabled and not status & 16):
            raise RuntimeError('CAN/enable không hợp lệ: '+str(status))
        data = self.samples['/motor_fb'][1]
        indices = [next((data['name'].index(n) for n in choices if n in data['name']), None)
                   for choices in [('left_wheel', 'left_wheel_joint'), ('right_wheel', 'right_wheel_joint')]]
        if any(i is None for i in indices):
            raise RuntimeError('Không nhận diện được tên hai bánh')
        poses = [data['position'][i] for i in indices]
        speeds = [data['velocity'][i] for i in indices]
        if not all(math.isfinite(v) for v in poses+speeds) or max(map(abs, speeds)) > 1.0:
            raise RuntimeError('Dữ liệu bánh lỗi hoặc tốc độ vượt 1 rad/s')
        if (self.count_publishers('/cmd_vel') != 1 or self.count_publishers('/cmd_torque') or
                self.count_publishers('/cmd_mit') or self.count_publishers('/motor_enable') != 1 or
                self.vel.get_subscription_count() != 1 or self.enable.get_subscription_count() != 1):
            raise RuntimeError('Có controller/publisher khác hoặc chưa nhận endpoint STM32')
        return poses

    def send(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.vel.publish(msg)

    def zero(self, seconds, enabled=False):
        end, next_send = time.monotonic()+seconds, 0.
        while time.monotonic() < end:
            self.spin()
            self.check(enabled)
            if time.monotonic() >= next_send:
                self.send()
                next_send = time.monotonic()+.02

    def run(self, turns, speed, radius, separation):
        end = time.monotonic()+25
        while time.monotonic() < end:
            self.spin()
            try:
                self.check()
                break
            except (RuntimeError, IndexError):
                pass
        self.check()
        self.enable.publish(Bool(data=False))
        self.phase = 'zero_prepare'
        self.zero(.6)
        self.enable.publish(Bool(data=True))
        self.zero(.8)
        self.check(True)
        results = []
        for wheel, name in [(0, 'TRÁI'), (1, 'PHẢI')]:
            self.phase = 'spin_left' if wheel == 0 else 'spin_right'
            start = self.check(True)
            target = turns*2*math.pi
            begin, next_send, next_print = time.monotonic(), 0., 0.
            v, w = wheel_command(wheel, speed, radius, separation)
            try:
                while True:
                    self.spin()
                    poses = self.check(True)
                    delta, other = poses[wheel]-start[wheel], poses[1-wheel]-start[1-wheel]
                    if travel_check(delta, other, target):
                        break
                    now = time.monotonic()
                    if now-begin > 32:
                        raise RuntimeError('Chưa đủ vòng sau 32 giây; không tự tăng lực/tốc độ')
                    if now >= next_send:
                        self.send(v, w)
                        next_send = now+.02
                    if now >= next_print:
                        print(f'Bánh {name}: {delta/(2*math.pi):.2f}/{turns:g} vòng', flush=True)
                        next_print = now+1
            finally:
                self.send()
            self.phase = 'zero_after_left' if wheel == 0 else 'zero_after_right'
            self.zero(.8, True)
            final = self.check(True)
            delta, other = final[wheel]-start[wheel], final[1-wheel]-start[1-wheel]
            travel_check(delta, other, target)
            results.append(dict(wheel=name, turns_encoder=delta/(2*math.pi),
                                duration_wall_s=time.monotonic()-begin))
            print(f'Đã dừng bánh {name}.', flush=True)
        return results

    def stop(self):
        self.phase = 'cleanup_zero_disable'
        end, next_send = time.monotonic()+.8, 0.
        while time.monotonic() < end:
            if time.monotonic() >= next_send:
                self.send()
                self.enable.publish(Bool(data=False))
                next_send = time.monotonic()+.05
            self.spin()
        item = self.samples.get('/motor_status')
        return bool(item and time.monotonic()-item[0] <= .35 and not item[1]['data'] & 16)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--turns', type=float, default=2)
    parser.add_argument('--wheel-speed', type=float, default=.5, help='rad/s, tối đa 0.5')
    args = parser.parse_args()
    if not 0 < args.turns <= 2 or not .4 <= args.wheel_speed <= .5:
        parser.error('Chỉ nhận 0 < turns <= 2 và 0.4 <= wheel-speed <= 0.5 rad/s')
    if os.environ.get('ROS_DOMAIN_ID') != '0':
        parser.error('Dùng ROS_DOMAIN_ID=0 như cấu hình MCU hiện tại')
    root = Path(__file__).resolve().parents[1]
    robot = yaml.safe_load((root/'src/pi5_slam_bringup/config/robot.yaml').read_text())['robot']
    radius, separation = float(robot['wheel_radius']), float(robot['wheel_separation'])
    if not all(math.isfinite(v) and v > 0 for v in (radius, separation)):
        raise RuntimeError('Geometry không hợp lệ')
    lock = open(f"{os.environ.get('XDG_RUNTIME_DIR', '/tmp')}/pi5-stm32-slam-{os.getuid()}-0.lock", 'w')
    fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
    if subprocess.run(['pgrep', '-x', 'micro_ros_agent'], capture_output=True).returncode == 0:
        raise RuntimeError('Agent đã chạy; dừng phiên cũ trước, không mở trùng')
    (root/'runs').mkdir(exist_ok=True)
    out = Path(tempfile.mkdtemp(prefix='visible_wheels_', dir=root/'runs'))
    print('Chỉ dùng khi HAI BÁNH ĐÃ KÊ, tay RC:AUTO, người trực cạnh xe.', flush=True)
    print('LOG:', out, flush=True)
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', 'micro_ros_agent'], text=True).strip()
    log = (out/'agent.log').open('x')
    agent = subprocess.Popen([prefix+'/lib/micro_ros_agent/micro_ros_agent', 'serial', '--dev',
        str(robot['stm32_port']), '--baudrate', '921600', '-v', '3'], stdout=log,
        stderr=subprocess.STDOUT, start_new_session=True)
    node = None
    result = dict(completed=False, hardware_file_modified=False, requested_turns=args.turns,
                  requested_wheel_speed_radps=args.wheel_speed)
    def interrupted(signum, frame):
        raise KeyboardInterrupt('Operator/timeout stopped test')
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        # Giữ context sống trong finally để Ctrl+C còn gửi được zero + disable.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = Probe(out)
        result['wheels'] = node.run(args.turns, args.wheel_speed, radius, separation)
        result['completed'] = True
    except BaseException as exc:
        result['error'] = type(exc).__name__+': '+str(exc)
        print(result['error'], flush=True)
    finally:
        if node is not None:
            try:
                result['disabled_observed'] = node.stop()
            except BaseException as exc:
                result['cleanup_error'] = str(exc)
            node.raw.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        (out/'RESULT.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        try:
            os.killpg(agent.pid, signal.SIGINT)
            agent.wait(timeout=4)
        except subprocess.TimeoutExpired:
            os.killpg(agent.pid, signal.SIGTERM)
            agent.wait(timeout=2)
        except ProcessLookupError:
            pass
        log.close()
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    raise SystemExit(0 if result['completed'] and result.get('disabled_observed') else 1)


if __name__ == '__main__':
    main()
