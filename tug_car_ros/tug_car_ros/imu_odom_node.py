#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

def q_normalize(w,x,y,z):
    n = math.sqrt(w*w + x*x + y*y + z*z) or 1.0
    return (w/n, x/n, y/n, z/n)

def yaw_to_quat(yaw):
    return q_normalize(math.cos(yaw/2.0), 0.0, 0.0, math.sin(yaw/2.0))

def yaw_from_quat(w,x,y,z):
    # ZYX yaw
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

class ImuOdomNode(Node):
    def __init__(self):
        super().__init__('imu_odom_node')

        # Params
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gyro_is_deg', False)   # 자이로 단위가 deg/s이면 True
        self.declare_parameter('two_d_mode', True)     # yaw만 사용

        self.imu_topic  = self.get_parameter('imu_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.gyro_is_deg = bool(self.get_parameter('gyro_is_deg').value)
        self.two_d_mode = bool(self.get_parameter('two_d_mode').value)

        # QoS: sensor_data (IMU 퍼블리셔와 일치)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(Imu, self.imu_topic, self.on_imu, sensor_qos)
        self.pub = self.create_publisher(Odometry, 'odom', sensor_qos)
        self.tf  = TransformBroadcaster(self)

        # state
        self.prev_t = None
        self.yaw = 0.0

        self.get_logger().info(f"[imu_odom] subscribe: {self.imu_topic}")
        self.get_logger().info(f"[imu_odom] publish  : /odom, TF {self.odom_frame}->{self.base_frame}")

    def on_imu(self, msg: Imu):
        # 시간 간격
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t is None:
            self.prev_t = now
        dt = max(1e-3, min(0.2, now - self.prev_t))
        self.prev_t = now

        # 1) orientation 유효성: covariance[0] == -1이면 유효하지 않음(관례)
        orientation_valid = (msg.orientation_covariance[0] >= 0.0)
        qw, qx, qy, qz = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        qw, qx, qy, qz = q_normalize(qw, qx, qy, qz)

        # 2) 자이로 단위 정규화
        gz = msg.angular_velocity.z
        if self.gyro_is_deg:
            gz = math.radians(gz)

        # 3) 2D 모드 처리: yaw 생성
        if self.two_d_mode:
            if orientation_valid and (qw != 0.0 or qx != 0.0 or qy != 0.0 or qz != 0.0):
                self.yaw = yaw_from_quat(qw, qx, qy, qz)
            else:
                # orientation이 없으면 자이로 z 적분
                self.yaw += gz * dt
            qw, qx, qy, qz = yaw_to_quat(self.yaw)

        # --- Odometry 메시지
        od = Odometry()
        od.header.stamp = msg.header.stamp if msg.header.stamp.sec != 0 else self.get_clock().now().to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id  = self.base_frame

        # 위치는 0 (휠 엔코더/비전 융합 전)
        od.pose.pose.position.x = 0.0
        od.pose.pose.position.y = 0.0
        od.pose.pose.position.z = 0.0

        od.pose.pose.orientation = Quaternion(w=qw, x=qx, y=qy, z=qz)
        od.twist.twist.angular = msg.angular_velocity  # 선택

        self.pub.publish(od)

        # TF(odom->base_link)
        t = TransformStamped()
        t.header.stamp = od.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id  = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = od.pose.pose.orientation
        self.tf.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

