#!/usr/bin/env python3
"""Kiểm chứng watchdog bằng timestamp MCU và CAN TXBTO; chỉ gửi torque zero."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

FIELDS = ['schema','timeout_ms','period_ms','event_seq','command_seq','command_tick',
          'detect_tick','queued_tick','left_tx_tick','right_tx_tick','detect_age_ms',
          'queued_age_ms','left_tx_age_ms','right_tx_age_ms','queued_mask','tx_mask','mode','ros_only']


def decode_debug(data):
    if len(data) < 40:
        raise ValueError('Firmware chưa có chẩn đoán watchdog 400 ms (cần 40 giá trị)')
    values = data[22:40]
    if not all(isinstance(v,(int,float)) and math.isfinite(v) and v==int(v) and 0<=v<=0xffffffff for v in values):
        raise ValueError('Invalid watchdog diagnostic integers')
    result = dict(zip(FIELDS,map(int,values)))
    if result['schema'] != 4001 or result['timeout_ms'] != 400 or result['ros_only'] != 0:
        raise ValueError('Wrong firmware schema/timeout/SBUS profile')
    return result


def validate_event(event, previous):
    if event['event_seq'] == previous:
        return False
    if event['queued_mask'] != 3 or event['tx_mask'] != 3:
        return False
    if event['command_seq']==0 or event['mode']!=1 or event['period_ms']!=2:
        raise ValueError('Event is not an enabled MIT/ROS watchdog event')
    ticks = ['detect_tick','queued_tick','left_tx_tick','right_tx_tick']
    ages = ['detect_age_ms','queued_age_ms','left_tx_age_ms','right_tx_age_ms']
    for tick,age in zip(ticks,ages):
        actual=(event[tick]-event['command_tick']) & 0xffffffff
        if actual != event[age]:
            raise ValueError('Inconsistent MCU tick/age: '+age)
    if not (400 < event['detect_age_ms'] <= event['queued_age_ms'] <=
            min(event['left_tx_age_ms'],event['right_tx_age_ms']) and
            max(event['left_tx_age_ms'],event['right_tx_age_ms']) <= 500):
        raise ValueError('Watchdog safe-frame transmission does not satisfy <=500ms')
    return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply',action='store_true',help='Chỉ ghi JSON sau hai sự kiện đạt và đã disable')
    parser.add_argument('--firmware-record',type=Path,help='MANIFEST có flash_verified=true của lần nạp này')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    record=None
    if args.apply:
        if args.firmware_record is None:parser.error('--apply cần --firmware-record')
        record=json.loads(args.firmware_record.read_text())
        if not (record.get('flash_verified') is True and record.get('timeout_ms')==400 and
                record.get('ros_only')==0 and record.get('motor_cybergear')==1):
            parser.error('Firmware record chưa được nạp/verify đúng profile')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Imu,JointState
    from std_msgs.msg import Bool,Float64MultiArray,UInt8
    from rosidl_runtime_py.convert import message_to_ordereddict
    if os.environ.get('ROS_DOMAIN_ID')!='0':parser.error('Dùng ROS_DOMAIN_ID=0')
    lock=open(f"{os.environ.get('XDG_RUNTIME_DIR','/tmp')}/pi5-stm32-slam-{os.getuid()}-0.lock",'w')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if subprocess.run(['pgrep','-x','micro_ros_agent'],capture_output=True).returncode==0:
        raise RuntimeError('Dừng agent/bringup cũ trước; không chạy trùng')
    out=Path(tempfile.mkdtemp(prefix='watchdog400_verify_',dir=root/'runs'))
    print('LOG:',out,flush=True)
    prefix=subprocess.check_output(['ros2','pkg','prefix','micro_ros_agent'],text=True).strip()
    log=(out/'agent.log').open('x')
    agent=subprocess.Popen([prefix+'/lib/micro_ros_agent/micro_ros_agent','serial','--dev',
        '/dev/stm32_robot','--baudrate','921600','-v','3'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)

    class Probe(Node):
        def __init__(self):
            super().__init__('watchdog400_verifier')
            self.samples={};self.raw=(out/'samples.jsonl').open('x');self.events=[]
            self.subs=[self.create_subscription(cls,topic,lambda msg,t=topic:self.receive(t,msg),qos_profile_sensor_data)
                for topic,cls in [('/imu/debug',Float64MultiArray),('/imu/data',Imu),('/motor_fb',JointState),('/motor_status',UInt8)]]
            self.zero=self.create_publisher(Float64MultiArray,'/cmd_torque',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
            self.enable=self.create_publisher(Bool,'/motor_enable',1)

        def receive(self,topic,msg):
            row=dict(topic=topic,received_monotonic_s=time.monotonic(),message=message_to_ordereddict(msg))
            self.samples[topic]=row;self.raw.write(json.dumps(row)+'\n')

        def spin(self):rclpy.spin_once(self,timeout_sec=.005)

        def check(self,enabled=False):
            now=time.monotonic()
            for topic,age in [('/motor_fb',.15),('/imu/data',.25),('/motor_status',.35),('/imu/debug',.35)]:
                row=self.samples.get(topic)
                if not row or now-row['received_monotonic_s']>age:raise RuntimeError('Missing/stale '+topic)
                if topic in ('/motor_fb','/imu/data'):
                    stamp=row['message']['header']['stamp']
                    dt=self.get_clock().now().nanoseconds*1e-9-stamp['sec']-stamp['nanosec']*1e-9
                    if not -.05<=dt<=.3:raise RuntimeError('Invalid source timestamp '+topic)
            status=self.samples['/motor_status']['message']['data']
            if status&3!=3 or status&12 or (enabled and not status&16):raise RuntimeError('CAN/enable status: '+str(status))
            speeds=self.samples['/motor_fb']['message']['velocity']
            if len(speeds)!=2 or not all(math.isfinite(v) and abs(v)<1 for v in speeds):raise RuntimeError('Unexpected wheel motion')
            if (self.count_publishers('/cmd_torque')!=1 or self.zero.get_subscription_count()!=1 or
                self.count_publishers('/cmd_vel') or self.count_publishers('/cmd_mit') or
                self.count_publishers('/motor_enable')!=1 or self.enable.get_subscription_count()!=1):
                raise RuntimeError('Conflicting or missing actuator endpoints')
            return decode_debug(self.samples['/imu/debug']['message']['data'])

        def hold_zero(self,duration,enabled=False):
            end,next_send=time.monotonic()+duration,0.
            while time.monotonic()<end:
                self.spin();self.check(enabled)
                if time.monotonic()>=next_send:
                    self.zero.publish(Float64MultiArray(data=[0.,0.]));next_send=time.monotonic()+.02

        def run(self):
            end=time.monotonic()+20
            while time.monotonic()<end:
                self.spin()
                if len(self.samples)==4:
                    try:
                        self.check();break
                    except (RuntimeError,ValueError):
                        pass
            self.check()
            self.enable.publish(Bool(data=False));self.hold_zero(.6)
            self.enable.publish(Bool(data=True));self.hold_zero(.8)
            self.check(True)
            for trial in range(2):
                self.hold_zero(.6,True)
                before=self.check(True)['event_seq']
                end=time.monotonic()+2.
                passed=False
                while time.monotonic()<end:
                    self.spin();event=self.check(True)
                    if validate_event(event,before):
                        self.events.append(event);passed=True
                        print('PASS event',trial+1,json.dumps(event),flush=True);break
                if not passed:raise RuntimeError('No new complete watchdog/CAN TX event; keep JSON blocked')

        def stop(self):
            end,next_send=time.monotonic()+.8,0.
            while time.monotonic()<end:
                if time.monotonic()>=next_send:
                    self.zero.publish(Float64MultiArray(data=[0.,0.]));self.enable.publish(Bool(data=False));next_send=time.monotonic()+.05
                self.spin()
            row=self.samples.get('/motor_status')
            return bool(row and time.monotonic()-row['received_monotonic_s']<=.35 and not row['message']['data']&16)

    def interrupt(signum,frame):raise KeyboardInterrupt
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=None;result=dict(passed=False,hardware_updated=False,zero_torque_only=True)
    try:
        node=Probe();node.run();result['passed']=True
    except BaseException as exc:result['error']=type(exc).__name__+': '+str(exc)
    finally:
        if node:
            try:result['disabled_observed']=node.stop()
            except BaseException as exc:result['cleanup_error']=str(exc)
            result['events']=node.events;node.raw.close();node.destroy_node()
        rclpy.shutdown()
        try:os.killpg(agent.pid,signal.SIGINT);agent.wait(timeout=4)
        except subprocess.TimeoutExpired:os.killpg(agent.pid,signal.SIGTERM);agent.wait(timeout=2)
        except ProcessLookupError:pass
        log.close()
    result['passed']=bool(result['passed'] and result.get('disabled_observed'))
    (out/'RESULT.json').write_text(json.dumps(result,indent=2))
    if result['passed'] and args.apply:
        sys.path.insert(0,str(root/'src/nino_real_rl'))
        from nino_real_rl.core import hardware_contract,load_hardware_contract
        path=root/'hardware.local.json';before=path.read_bytes();data=json.loads(before)
        measured_max_ms=max(max(e['left_tx_age_ms'],e['right_tx_age_ms']) for e in result['events'])
        data.update(watchdog_verified=True,verified_watchdog_seconds=measured_max_ms/1000.,
                    configured_watchdog_seconds=.4,
                    watchdog_verification_evidence=str(out/'RESULT.json'),
                    firmware_verification_record=str(args.firmware_record.resolve()))
        hardware_contract(data)
        shutil.copy2(path,out/'hardware.before.json')
        fd,name=tempfile.mkstemp(prefix='.hardware.watchdog400.',suffix='.tmp',dir=root)
        with os.fdopen(fd,'w') as f:
            f.write(json.dumps(data,indent=2)+'\n');f.flush();os.fsync(f.fileno())
        os.chmod(name,path.stat().st_mode&0o777);os.replace(name,path)
        load_hardware_contract(str(path))
        result['hardware_updated']=True
        (out/'RESULT.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__':main()
