"""Bàn phím có timeout hoạt động; chỉ phát /teleop/cmd_vel, không bật motor."""
from math import isfinite
from time import monotonic

KEYS={'i':(1,0),',':(-1,0),'j':(0,1),'l':(0,-1),
      'u':(1,1),'o':(1,-1),'m':(-1,-1),'.':(-1,1)}


class KeyboardCommand:
    def __init__(self,speed=.50,turn=.40,key_timeout=.60):
        if not all(isfinite(v) and v>0 for v in (speed,turn,key_timeout)):
            raise ValueError('Keyboard speed/turn/timeout must be finite and positive')
        self.speed,self.turn,self.timeout=speed,turn,key_timeout
        self.last_key=None;self.value=(0.,0.)

    def press(self,key,now):
        direction=KEYS.get(key,(0,0))
        self.value=(direction[0]*self.speed,direction[1]*self.turn)
        self.last_key=now

    def current(self,now):
        if self.last_key is None or not 0<=now-self.last_key<self.timeout:return (0.,0.)
        return self.value


def read_key(fd):
    """Đọc byte sau select; không chờ bộ giải mã Unicode hoặc đệm TextIO."""
    import os
    data=os.read(fd,1)
    return None if not data else data.decode('ascii',errors='ignore')


def main(args=None):
    import select
    import sys
    import termios
    import tty
    import rclpy
    from geometry_msgs.msg import Twist
    if not sys.stdin.isatty():raise SystemExit('Cần terminal tương tác; nếu SSH dùng ssh -t.')
    rclpy.init(args=args);node=rclpy.create_node('nino_teleop_keyboard')
    speed=node.declare_parameter('max_linear_speed',.50).value
    turn=node.declare_parameter('max_angular_speed',.40).value
    hold=node.declare_parameter('key_hold_timeout',.60).value
    topic=node.declare_parameter('input_topic','/teleop/cmd_vel').value
    if topic in ('/cmd_vel','/cmd_torque','/cmd_mit'):
        raise ValueError('Keyboard must publish to guarded input, not firmware command topics')
    publisher=node.create_publisher(Twist,topic,1)
    state=KeyboardCommand(speed,turn,hold)
    settings=termios.tcgetattr(sys.stdin)
    def send(value):
        message=Twist();message.linear.x=float(value[0]);message.angular.z=float(value[1])
        publisher.publish(message)
    print('Teleop: i tiến, , lùi, j trái, l phải; u/o/m/. đi chéo; k hoặc SPACE dừng; q/Ctrl+C thoát.',flush=True)
    print(f'Tối đa {state.speed:.2f} m/s, {state.turn:.2f} rad/s. Giữ/nhấn lặp phím; ngừng phím {state.timeout:.2f} s tự gửi zero.',flush=True)
    try:
        tty.setraw(sys.stdin.fileno());last_publish=-1.;last_value=None
        while rclpy.ok():
            ready,_,_=select.select([sys.stdin],[],[],.02)
            now=monotonic()
            if ready:
                key=read_key(sys.stdin.fileno())
                if key in ('q','\x03',None):break
                state.press(key,now)
            value=state.current(now)
            if value!=last_value or now-last_publish>=.10:
                send(value);last_publish=now;last_value=value
            rclpy.spin_once(node,timeout_sec=0)
    except KeyboardInterrupt:pass
    finally:
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
        if rclpy.ok():send((0.,0.))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
