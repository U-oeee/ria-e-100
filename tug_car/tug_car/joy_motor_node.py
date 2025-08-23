# motor_control_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import serial

class Motor:
    """RS232 모터 드라이버 래퍼"""
    SPEED_MODE = 1

    def __init__(self, port, baudrate=57600, parity='N', node: Node=None):
        self.node = node
        self.ser = None
        parity_map = {
            'N': serial.PARITY_NONE,
            'E': serial.PARITY_EVEN,
            'O': serial.PARITY_ODD,
        }
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=parity_map.get(parity.upper(), serial.PARITY_NONE),
                timeout=0.1
            )
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f'Motor serial open failed ({port}): {e}')
            self.ser = None
        self.mode = self.SPEED_MODE

    def _write_hex(self, hex_str: str):
        if not self.ser:
            return
        try:
            self.ser.write(bytearray.fromhex(hex_str))
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f'Serial write failed: {e}')

    def send_serial(self, *data):
        if not self.ser:
            return
        msg = " ".join("{:02X}".format(d) for d in data)
        self._write_hex(msg)

    def send_serial_checksum(self, checksum=True, *data):
        if not self.ser:
            return
        if checksum:
            check = sum(data) % 256
            self.send_serial(*(list(data) + [check]))
        else:
            self.send_serial(*data)

    def run(self):
        # 드라이버 러닝 커맨드 (제조사 프로토콜 그대로)
        self._write_hex("00 00 01 01")

    def stop(self):
        self._write_hex("00 00 00 00")

    def set_speed_mode(self, acc_time=1.0, dcc_time=1.0):
        if not self.ser:
            return
        acc = int(acc_time * 10)
        dcc = int(dcc_time * 10)
        self.send_serial_checksum(True, 0x02, 0x00, 0xC4, 0xC6)
        self.send_serial_checksum(True, 0x0A, acc, dcc)

    def set_rpm(self, rpm: int):
        # 장치 스펙: RPM → 내부 단위로 스케일 (기존 코드 유지)
        rpm_val = int(rpm * 8192 / 3000)
        high_byte = (rpm_val >> 8) & 0xFF
        low_byte = rpm_val & 0xFF
        self.send_serial_checksum(True, 0x06, high_byte, low_byte)

    def close(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # -------- Parameters --------
        self.declare_parameter('left_port', '/dev/left_wheel_usb')
        self.declare_parameter('right_port', '/dev/right_wheel_usb')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('parity', 'N')  # 'N','E','O'
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('pub_front_topic', 'motor_vel/front')
        self.declare_parameter('pub_rear_topic',  'motor_vel/rear')
        self.declare_parameter('linear_speed_init', 0.5)   # 0.3~0.8 권장
        self.declare_parameter('angular_speed_init', 0.3)  # 0.1~0.5 권장
        self.declare_parameter('rpm_scale', 100.0)         # speed(±1.0)→RPM 스케일
        self.declare_parameter('deadzone', 0.05)           # 조이스틱 데드존

        left_port  = self.get_parameter('left_port').value
        right_port = self.get_parameter('right_port').value
        baudrate   = int(self.get_parameter('baudrate').value)
        parity     = str(self.get_parameter('parity').value)
        joy_topic  = str(self.get_parameter('joy_topic').value)
        front_topic= str(self.get_parameter('pub_front_topic').value)
        rear_topic = str(self.get_parameter('pub_rear_topic').value)

        self.linear_speed  = float(self.get_parameter('linear_speed_init').value)
        self.angular_speed = float(self.get_parameter('angular_speed_init').value)
        self.rpm_scale     = float(self.get_parameter('rpm_scale').value)
        self.deadzone      = float(self.get_parameter('deadzone').value)

        # -------- Motors --------
        self.left_motor  = Motor(left_port,  baudrate=baudrate, parity=parity, node=self)
        self.right_motor = Motor(right_port, baudrate=baudrate, parity=parity, node=self)
        self.left_motor.set_speed_mode(acc_time=1.0, dcc_time=1.0)
        self.right_motor.set_speed_mode(acc_time=1.0, dcc_time=1.0)

        # -------- Publishers --------
        self.pub_front = self.create_publisher(Float32MultiArray, front_topic, 10)
        self.pub_rear  = self.create_publisher(Float32MultiArray, rear_topic,  10)

        # -------- Subscriber --------
        self.subscription = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        self.get_logger().info(
            f'Motor teleop ready: L={left_port}, R={right_port}, baud={baudrate}, parity={parity}, '
            f'joy="{joy_topic}", pubs=("{front_topic}","{rear_topic}")'
        )

    @staticmethod
    def _apply_deadzone(v: float, dz: float) -> float:
        return 0.0 if abs(v) < dz else v

    def joy_callback(self, msg: Joy):
        # 매핑(네 기존 코드와 동일)
        angular_adjust     = self._apply_deadzone(msg.axes[0], self.deadzone)  # 속도 가감
        linear_adjust      = self._apply_deadzone(msg.axes[1], self.deadzone)  # 속도 가감
        right_joystick_lr  = self._apply_deadzone(msg.axes[4], self.deadzone)  # 좌우 회전
        right_joystick_ud  = self._apply_deadzone(msg.axes[3], self.deadzone)  # 전후 이동

        # 속도 가감
        if angular_adjust == 1.0:
            self.angular_speed *= 1.1
        elif angular_adjust == -1.0:
            self.angular_speed *= 0.9
        self.angular_speed = max(0.1, min(self.angular_speed, 0.5))

        if linear_adjust == 1.0:
            self.linear_speed *= 1.1
        elif linear_adjust == -1.0:
            self.linear_speed *= 0.9
        self.linear_speed = max(0.3, min(self.linear_speed, 0.8))

        # 조이스틱 → 좌/우 속도
        forward = right_joystick_ud * self.angular_speed
        turn    = -right_joystick_lr * self.linear_speed
        left_speed  = forward + turn     # [-1..1] 근사
        right_speed = forward - turn

        # speed → RPM 명령(스케일링, 부호 유지)
        left_rpm_cmd  = int(-left_speed  * self.rpm_scale)
        right_rpm_cmd = int(-right_speed * self.rpm_scale)

        # RS232 전송
        self.left_motor.set_rpm(left_rpm_cmd)
        self.right_motor.set_rpm(right_rpm_cmd)
        self.left_motor.run()
        self.right_motor.run()

        # ROS로도 퍼블리시 → e100_odom이 체인으로 사용
        msg_front = Float32MultiArray()
        msg_rear  = Float32MultiArray()
        # 메카넘/차동 양쪽 모두 앞/뒤 같은 값으로 퍼블리시(필요시 기구학에 맞게 변경)
        msg_front.data = [float(left_rpm_cmd), float(right_rpm_cmd)]
        msg_rear.data  = [float(left_rpm_cmd), float(right_rpm_cmd)]
        self.pub_front.publish(msg_front)
        self.pub_rear.publish(msg_rear)

        # 디버그(너무 시끄러우면 주석)
        self.get_logger().info(
            f'joy: axes={list(map(lambda x: round(x,2), msg.axes))} | '
            f'ang={self.angular_speed:.2f} lin={self.linear_speed:.2f} | '
            f'cmd RPM L={left_rpm_cmd} R={right_rpm_cmd}'
        )

    def destroy_node(self):
        try:
            self.left_motor.stop()
            self.right_motor.stop()
            self.left_motor.close()
            self.right_motor.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

