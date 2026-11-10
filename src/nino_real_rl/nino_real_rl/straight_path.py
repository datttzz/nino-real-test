"""Đường thẳng trong odom: RViz gửi đích, Nav2 FollowPath tạo lệnh vận tốc.

Không phát /cmd_vel, /cmd_torque hoặc motor_enable. Không giả map/AMCL.
"""
from collections import deque
from copy import deepcopy
from math import atan2, ceil, cos, hypot, isfinite, sin

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def line_points(start, end, spacing=.05, max_distance=3.):
    """Điểm đầu/cuối giữ đúng tọa độ odom, hướng cuối trùng tiếp tuyến đường."""
    if len(start)!=2 or len(end)!=2 or not all(isfinite(v) for v in (*start,*end,spacing,max_distance)):
        raise ValueError('Tọa độ/khoảng cách đường không hợp lệ')
    distance=hypot(end[0]-start[0],end[1]-start[1])
    if spacing<=0 or max_distance<=0 or not .05<=distance<=max_distance:
        raise ValueError(f'Chọn đích cách xe từ 0,05 đến {max_distance:g} m')
    count=ceil(distance/spacing);yaw=atan2(end[1]-start[1],end[0]-start[0])
    return [(start[0]+(end[0]-start[0])*i/count,
             start[1]+(end[1]-start[1])*i/count,yaw) for i in range(count+1)]


class StraightPath(Node):
    def __init__(self):
        super().__init__('straight_path')
        self.declare_parameter('max_distance',3.)
        self.max_distance=float(self.get_parameter('max_distance').value)
        if not isfinite(self.max_distance) or self.max_distance<.05:
            raise ValueError('max_distance phải >=0,05 m')
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.client=ActionClient(self,FollowPath,'follow_path')
        self.reference=None;self.active=False;self.goal_handle=None;self.cancel_pending=False
        self.trace=deque(maxlen=3000);self.last_trace_stamp=None
        self.reference_pub=self.create_publisher(Path,'/straight/reference_path',1)
        self.trace_pub=self.create_publisher(Path,'/straight/odom_trace',1)
        self.create_subscription(PoseStamped,'/goal_pose',self.on_goal,1)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_service(Trigger,'/straight/stop',self.on_stop)
        self.create_timer(.1,self.publish_paths)
        self.get_logger().info('Không dùng map/AMCL. RViz Fixed Frame=odom; dùng 2D Goal Pose. '
                               'Đường tham chiếu xanh; vết xe odometry đỏ. Chưa có goal thì không chạy.')

    def on_goal(self,message):
        if self.active:
            self.get_logger().warning('Đang bám đường; dừng hoặc chờ kết thúc trước khi chọn đích khác')
            return
        try:
            if message.header.frame_id!='odom':
                raise ValueError('Goal phải ở odom: đặt RViz Fixed Frame=odom')
            if not self.client.server_is_ready():
                raise ValueError('FollowPath chưa active; đợi Nav2 khởi động')
            transform=self.tf.lookup_transform('odom','base_footprint',Time())
            stamp=Time.from_msg(transform.header.stamp).nanoseconds*1e-9
            age=self.get_clock().now().nanoseconds*1e-9-stamp
            if not 0<=age<=.25:
                raise ValueError('TF odom→base_footprint chưa mới')
            p=transform.transform.translation;target=message.pose.position
            points=line_points((p.x,p.y),(target.x,target.y),max_distance=self.max_distance)
            path=Path();path.header.frame_id='odom';path.header.stamp=self.get_clock().now().to_msg()
            for x,y,yaw in points:
                pose=PoseStamped();pose.header=path.header
                pose.pose.position.x=float(x);pose.pose.position.y=float(y)
                pose.pose.orientation.z=sin(yaw/2);pose.pose.orientation.w=cos(yaw/2)
                path.poses.append(pose)
            goal=FollowPath.Goal(path=path,controller_id='FollowPath',
                                 goal_checker_id='general_goal_checker',progress_checker_id='progress_checker')
            self.reference=path;self.trace.clear();self.last_trace_stamp=None
            self.active=True;self.cancel_pending=False
            self.client.send_goal_async(goal).add_done_callback(self.on_accepted)
            self.get_logger().info(f'Đường odom ({p.x:.2f},{p.y:.2f}) → ({target.x:.2f},{target.y:.2f})')
        except Exception as exc:
            self.active=False
            self.get_logger().error(str(exc))

    def on_accepted(self,future):
        try:
            handle=future.result()
            if not handle.accepted:
                raise ValueError('Nav2 từ chối FollowPath')
            self.goal_handle=handle
            handle.get_result_async().add_done_callback(self.on_result)
            if self.cancel_pending:handle.cancel_goal_async()
        except Exception as exc:
            self.active=False;self.goal_handle=None
            self.get_logger().error(str(exc))

    def on_result(self,future):
        self.active=False;self.goal_handle=None
        try:
            response=future.result()
            self.get_logger().info(f'FollowPath status={response.status}, '
                f'error_code={response.result.error_code}: {response.result.error_msg}')
        except Exception as exc:
            self.get_logger().error(str(exc))

    def on_stop(self,request,response):
        self.cancel_pending=True
        if self.goal_handle is not None:self.goal_handle.cancel_goal_async()
        response.success=True
        response.message='Đã yêu cầu hủy FollowPath; đây không phải dừng khẩn phần cứng'
        return response

    def on_odom(self,message):
        if message.header.frame_id!='odom' or message.child_frame_id!='base_footprint':return
        stamp=Time.from_msg(message.header.stamp).nanoseconds*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        if not 0<=age<=.25 or (self.last_trace_stamp is not None and stamp-self.last_trace_stamp<.05):return
        p=message.pose.pose.position
        if not all(isfinite(v) for v in (p.x,p.y,p.z)):return
        pose=PoseStamped();pose.header=deepcopy(message.header);pose.pose=deepcopy(message.pose.pose)
        self.trace.append(pose);self.last_trace_stamp=stamp

    def publish_paths(self):
        if self.reference is not None:
            # Làm mới delivery của đường cố định, không sửa timestamp cảm biến.
            self.reference.header.stamp=self.get_clock().now().to_msg()
            self.reference_pub.publish(self.reference)
        if self.trace:
            path=Path();path.header.frame_id='odom';path.header.stamp=self.get_clock().now().to_msg()
            path.poses=list(self.trace);self.trace_pub.publish(path)


def main(args=None):
    rclpy.init(args=args);node=StraightPath()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
