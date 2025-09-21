#!/usr/bin/env python3
import rclpy, threading, queue
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import serial

class Ximu3Publisher(Node):
    def __init__(self):
        super().__init__('ximu3_publisher')

        # Params
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame', 'imu_link')
        self.declare_parameter('publish_tf', True)

        self.serial_port = self.get_parameter('serial_port').value
        self.serial_baud = self.get_parameter('serial_baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # QoS: SensorData
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pub = self.create_publisher(Imu, 'imu/data', sensor_qos)
        self.br = TransformBroadcaster(self) if self.publish_tf else None

        # Serial: non-blocking 느낌을 위해 timeout을 매우 짧게
        self.ser = serial.Serial(self.serial_port, self.serial_baud, timeout=0.001)
        self.get_logger().info(f"Listening on {self.serial_port} @ {self.serial_baud}")

        # Buffers
        self.last_quat = None
        self.last_gyr = None
        self.last_acc = None

        # Reader thread & queue
        self.q = queue.Queue(maxsize=200)
        self._stop = False
        t = threading.Thread(target=self.reader_loop, daemon=True)
        t.start()

        # Timer: 큐 소비(가능하면 더 빠르게)
        self.create_timer(0.005, self.process_lines)  # 200 Hz tick

    def reader_loop(self):
        # 라인 단위 파서
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
                    try:
                        self.q.put_nowait(line.decode(errors='ignore').strip())
                    except queue.Full:
                        # 가장 오래된 것 버리고 최신 유지
                        try: self.q.get_nowait()
                        except queue.Empty: pass
                        self.q.put_nowait(line.decode(errors='ignore').strip())
            except Exception as e:
                self.get_logger().warn(f"reader error: {e}")

    def process_lines(self):
        # 큐에 있는 만큼 처리 → 최신 상태로 빠르게 수렴
        try:
            while not self.q.empty():
                line = self.q.get_nowait()
                self.parse_line(line)
            if self.last_quat or self.last_gyr or self.last_acc:
                self.publish_imu()
        except Exception as e:
            self.get_logger().warn(f"process error: {e}")

    def parse_line(self, line: str):
        parts = line.split(',')
        if len(parts) < 2:
            return
        rec_type = parts[0]
        if rec_type == 'Q' and len(parts) >= 6:
            _, _, qx, qy, qz, qw = parts[:6]
            self.last_quat = (float(qw), float(qx), float(qy), float(qz))
        elif rec_type == 'I' and len(parts) >= 9:
            _, _, gx, gy, gz, ax, ay, az = parts[:9]
            self.last_gyr = (float(gx), float(gy), float(gz))
            self.last_acc = (float(ax), float(ay), float(az))

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        if self.last_quat:
            qw, qx, qy, qz = self.last_quat
            msg.orientation.w = qw; msg.orientation.x = qx; msg.orientation.y = qy; msg.orientation.z = qz
        else:
            msg.orientation_covariance[0] = -1.0

        if self.last_gyr:
            gx, gy, gz = self.last_gyr
            msg.angular_velocity.x = gx; msg.angular_velocity.y = gy; msg.angular_velocity.z = gz
        else:
            msg.angular_velocity_covariance[0] = -1.0

        if self.last_acc:
            ax, ay, az = self.last_acc
            msg.linear_acceleration.x = ax; msg.linear_acceleration.y = ay; msg.linear_acceleration.z = az
        else:
            msg.linear_acceleration_covariance[0] = -1.0

        self.pub.publish(msg)

        if self.br and self.last_quat:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.parent_frame
            t.child_frame_id = self.child_frame
            t.transform.rotation.w = msg.orientation.w
            t.transform.rotation.x = msg.orientation.x
            t.transform.rotation.y = msg.orientation.y
            t.transform.rotation.z = msg.orientation.z
            self.br.sendTransform(t)

    def destroy_node(self):
        self._stop = True
        try: self.ser.close()
        except: pass
        super().destroy_node()

def main():
    rclpy.init()
    node = Ximu3Publisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
