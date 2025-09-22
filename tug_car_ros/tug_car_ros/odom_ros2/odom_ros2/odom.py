#!/usr/bin/env python3
import math, threading, queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

import serial

def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw/2.0),
        w=math.cos(yaw/2.0)
    )

def quat_to_yaw(q: Quaternion) -> float:
    # normalize
    n = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z) or 1.0
    w,x,y,z = q.w/n, q.x/n, q.y/n, q.z/n
    # ZYX yaw
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

class ImuEncoderOdomNode(Node):
    def __init__(self):
        super().__init__('imu_encoder_odom_node')

        # ---------------- Parameters ----------------
        # IMU
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('imu_frame', 'imu_link')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame',  'imu_link')
        self.declare_parameter('publish_tf_imu', True)

        # Encoder topics
        self.declare_parameter('left_ticks_topic',  '/left_ticks')
        self.declare_parameter('right_ticks_topic', '/right_ticks')
        self.declare_parameter('left_invert',  False)
        self.declare_parameter('right_invert', False)
        self.declare_parameter('ticks_per_rev', 4220.0)
        self.declare_parameter('wheel_radius',  0.05)   # [m]
        self.declare_parameter('wheel_base',    0.30)   # [m]

        # Fusion
        self.declare_parameter('alpha_imu', 0.02)  # 0: encoder only, 1: imu only

        # Frames
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Read params
        self.serial_port   = self.get_parameter('serial_port').value
        self.serial_baud   = int(self.get_parameter('serial_baud').value)
        self.imu_frame     = self.get_parameter('imu_frame').value
        self.parent_frame  = self.get_parameter('parent_frame').value
        self.child_frame   = self.get_parameter('child_frame').value
        self.publish_tf_imu= bool(self.get_parameter('publish_tf_imu').value)

        self.left_topic    = self.get_parameter('left_ticks_topic').value
        self.right_topic   = self.get_parameter('right_ticks_topic').value
        self.left_inv      = bool(self.get_parameter('left_invert').value)
        self.right_inv     = bool(self.get_parameter('right_invert').value)
        self.N             = float(self.get_parameter('ticks_per_rev').value)
        self.R             = float(self.get_parameter('wheel_radius').value)
        self.L             = float(self.get_parameter('wheel_base').value)

        self.alpha_imu     = float(self.get_parameter('alpha_imu').value)
        self.odom_frame    = self.get_parameter('odom_frame').value
        self.base_frame    = self.get_parameter('base_frame').value

        # ---------------- QoS ----------------
        # 요청대로: Reliable로 통일 (echo 기본 QoS와 호환)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )

        sensor_qos = reliable_qos  # IMU 퍼블리셔도 Reliable로

        # ---------------- Publishers/Subscribers ----------------
        self.imu_pub = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', reliable_qos)
        self.tf_br = TransformBroadcaster(self)

        self.create_subscription(Int32, self.left_topic,  self.cb_left,  reliable_qos)
        self.create_subscription(Int32, self.right_topic, self.cb_right, reliable_qos)

        # ---------------- IMU Serial (thread + queue) ----------------
        try:
            self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=0.001)
            self.get_logger().info(f"[IMU] Listening on {self.serial_port} @ {self.serial_baud}")
        except Exception as e:
            self.get_logger().error(f"[IMU] Serial open failed: {e}")
            raise

        self.q = queue.Queue(maxsize=200)
        self._stop = False
        t = threading.Thread(target=self.reader_loop, daemon=True)
        t.start()
        self.create_timer(0.005, self.process_lines)   # 200 Hz tick

        # ---------------- State ----------------
        self.last_quat = None
        self.last_gyr  = None
        self.last_acc  = None
        self.imu_yaw   = None

        self.last_left  = None
        self.last_right = None
        self._left_now  = None
        self._right_now = None

        self.x = 0.0; self.y = 0.0; self.th = 0.0
        self.last_time = self.get_clock().now()

        self.get_logger().info(f"[ODOM] R={self.R:.3f} L={self.L:.3f} N={self.N:.1f} alpha_imu={self.alpha_imu:.2f}")

    # ---------------- IMU read path ----------------
    def reader_loop(self):
        buf = b""
        while not self._stop:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    if not line:
                        continue
                    s = line.decode(errors='ignore').strip()
                    try:
                        self.q.put_nowait(s)
                    except queue.Full:
                        try: self.q.get_nowait()
                        except queue.Empty: pass
                        self.q.put_nowait(s)
            except Exception as e:
                self.get_logger().warn(f"[IMU] reader error: {e}")

    def process_lines(self):
        try:
            updated = False
            while not self.q.empty():
                line = self.q.get_nowait()
                updated = self.parse_imu_line(line) or updated
            if updated:
                self.publish_imu()
        except Exception as e:
            self.get_logger().warn(f"[IMU] process error: {e}")

    def parse_imu_line(self, line: str):
        parts = line.split(',')
        if len(parts) < 2:
            return False
        rec_type = parts[0]
        if rec_type == 'Q' and len(parts) >= 6:
            # Q,timestamp,qx,qy,qz,qw
            _, _, qx, qy, qz, qw = parts[:6]
            qw, qx, qy, qz = float(qw), float(qx), float(qy), float(qz)
            self.last_quat = (qw, qx, qy, qz)
            self.imu_yaw = quat_to_yaw(Quaternion(w=qw, x=qx, y=qy, z=qz))
            return True
        elif rec_type == 'I' and len(parts) >= 9:
            # I,timestamp,gx,gy,gz,ax,ay,az
            _, _, gx, gy, gz, ax, ay, az = parts[:9]
            self.last_gyr = (float(gx), float(gy), float(gz))
            self.last_acc = (float(ax), float(ay), float(az))
            return True
        return False

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame

        if self.last_quat:
            qw, qx, qy, qz = self.last_quat
            msg.orientation.w = qw
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
        else:
            msg.orientation_covariance[0] = -1.0

        if self.last_gyr:
            gx, gy, gz = self.last_gyr
            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz
        else:
            msg.angular_velocity_covariance[0] = -1.0

        if self.last_acc:
            ax, ay, az = self.last_acc
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az
        else:
            msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)

        if self.publish_tf_imu and self.last_quat:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.parent_frame
            t.child_frame_id  = self.child_frame
            t.transform.rotation.w = msg.orientation.w
            t.transform.rotation.x = msg.orientation.x
            t.transform.rotation.y = msg.orientation.y
            t.transform.rotation.z = msg.orientation.z
            self.tf_br.sendTransform(t)

    # ---------------- Encoder path ----------------
    def cb_left(self, msg: Int32):
        v = -msg.data if self.left_inv else msg.data
        self._left_now = v
        if self.last_left is None:
            self.last_left = v
        self.try_integrate()

    def cb_right(self, msg: Int32):
        v = -msg.data if self.right_inv else msg.data
        self._right_now = v
        if self.last_right is None:
            self.last_right = v
        self.try_integrate()

    def try_integrate(self):
        if self._left_now is None or self._right_now is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:  # 첫 샘플 등
            self.last_time = now
            return

        dL_ticks = self._left_now  - self.last_left
        dR_ticks = self._right_now - self.last_right
        self.last_left  = self._left_now
        self.last_right = self._right_now
        self.last_time  = now

        # tick -> distance
        dL = 2.0*math.pi*self.R * (dL_ticks / self.N)
        dR = 2.0*math.pi*self.R * (dR_ticks / self.N)

        ds      = 0.5*(dL + dR)
        dth_enc = (dR - dL) / self.L

        # IMU yaw 융합(보완필터)
        th_pred = self.th + dth_enc
        if self.imu_yaw is not None and self.alpha_imu > 0.0:
            err = math.atan2(math.sin(self.imu_yaw - th_pred), math.cos(self.imu_yaw - th_pred))
            self.th = th_pred + self.alpha_imu * err
        else:
            self.th = th_pred

        # midpoint integration
        self.x += ds * math.cos(self.th - 0.5*dth_enc)
        self.y += ds * math.sin(self.th - 0.5*dth_enc)

        v = ds/dt
        w = dth_enc/dt
        self.publish_odom(now, v, w)

    def publish_odom(self, stamp, v, w):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.th)

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_br.sendTransform(t)

    # --------------- shutdown ---------------
    def destroy_node(self):
        self._stop = True
        try:
            if hasattr(self, 'ser') and self.ser:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = ImuEncoderOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

