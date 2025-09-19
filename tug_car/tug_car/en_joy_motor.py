#!/usr/bin/env python3 encoder + joystick control
import math
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial

ENC_RESOLUTION = 4220 # 메뉴얼: 8192 tick = 1 rev = 2π rad


class Motor:
    """Wrapper class for motor driver serial (RS232) communication"""

    SPEED_MODE = 1

    def __init__(self, port, baudrate=57600, parity=serial.PARITY_NONE, timeout=0.15):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=parity,
            timeout=timeout,
            write_timeout=0.15
        )
        self.mode = self.SPEED_MODE
        self._lock = threading.Lock()

    # ---------------- low-level ----------------
    def send_serial(self, *data):
        msg = ""
        for d in data:
            msg += "{:02X} ".format(d)
        with self._lock:
            self.ser.write(bytearray.fromhex(msg.strip()))
            self.ser.flush()

    def send_serial_checksum(self, checksum=True, *data):
        if checksum:
            check = sum(data) % 256
            self.send_serial(*(list(data) + [check]))
        else:
            self.send_serial(*data)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # ---------------- driver commands ----------------
    def run(self):
        with self._lock:
            self.ser.write(bytearray.fromhex("00 00 01 01"))
            self.ser.flush()

    def stop(self):
        with self._lock:
            self.ser.write(bytearray.fromhex("00 00 00 00"))
            self.ser.flush()

    def set_speed_mode(self, acc_time=1, dcc_time=1):
        acc = int(acc_time * 10)
        dcc = int(dcc_time * 10)
        self.send_serial_checksum(True, 0x02, 0x00, 0xC4, 0xC6)
        self.send_serial_checksum(True, 0x0A, acc, dcc)

    def set_rpm(self, rpm):
        print(f"[Motor Debug] Set RPM: {rpm}")
        rpm_val = int(rpm * 8192 / 3000)
        high_byte = (rpm_val >> 8) & 0xFF
        low_byte = rpm_val & 0xFF
        self.send_serial_checksum(True, 0x06, high_byte, low_byte)

    # ---------------- encoder reading ----------------
    def _hexdump(self, data, maxlen=64):
        return ' '.join(f'{b:02X}' for b in data[:maxlen])

    def read_position_feedback_ticks(self):
        """
        모니터링 쿼리 전송 후 응답에서
        E8(상위16bit), E9(하위16bit)를 조합해 32-bit tick 반환
        """
        try:
            # flush input buffer
            with self._lock:
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass

            # query 1: 3-byte
            self.send_serial(0x80, 0x00, 0x80)

            start = time.time()
            buf = bytearray()
            while (time.time() - start) < 0.06:
                with self._lock:
                    n = self.ser.in_waiting if hasattr(self.ser, "in_waiting") else 0
                    if n:
                        buf += self.ser.read(n)
                if len(buf) >= 16 and (0xE8 in buf or 0xE9 in buf):
                    break
                time.sleep(0.002)

            # query 2: fallback 4-byte
            if len(buf) < 6 or (0xE8 not in buf and 0xE9 not in buf):
                self.send_serial(0x80, 0x00, 0x00, 0x80)
                start = time.time()
                buf = bytearray()
                while (time.time() - start) < 0.06:
                    with self._lock:
                        n = self.ser.in_waiting if hasattr(self.ser, "in_waiting") else 0
                        if n:
                            buf += self.ser.read(n)
                    if len(buf) >= 16 and (0xE8 in buf or 0xE9 in buf):
                        break
                    time.sleep(0.002)

            if len(buf) < 6:
                return None

            # parse addr+data pairs
            e8_val, e9_val = None, None
            i = 0
            L = len(buf)
            while i + 2 < L:
                addr = buf[i]
                d1 = buf[i + 1]
                d0 = buf[i + 2]
                if addr == 0xE8:
                    e8_val = (d1 << 8) | d0
                    i += 3
                    continue
                if addr == 0xE9:
                    e9_val = (d1 << 8) | d0
                    i += 3
                    continue
                i += 1

            if e8_val is None or e9_val is None:
                return None

            raw32 = ((e8_val & 0xFFFF) << 16) | (e9_val & 0xFFFF)

            # signed 32bit
            if raw32 & 0x80000000:
                raw32 = -((~raw32 & 0xFFFFFFFF) + 1)

            return raw32

        except Exception as e:
            print(f"[Encoder Error] {e}")
            return None

    def read_position_feedback_rad(self):
        ticks = self.read_position_feedback_ticks()
        if ticks is None:
            return None
        return (2.0 * math.pi * ticks) / ENC_RESOLUTION


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # speed limits
        self.default_angular_speed = 0.3
        self.default_linear_speed = 0.5
        self.angular_speed = self.default_angular_speed
        self.linear_speed = self.default_linear_speed

        # motors
        self.left_motor = Motor('/dev/left_wheel_usb')
        self.right_motor = Motor('/dev/right_wheel_usb')
        self.left_motor.set_speed_mode(acc_time=1, dcc_time=1)
        self.right_motor.set_speed_mode(acc_time=1, dcc_time=1)

        # joystick subscription
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # encoder polling
        self.encoder_timer = self.create_timer(0.05, self.poll_and_print_encoders)

    # ---------------- joystick ----------------
    def joy_callback(self, msg):
        self.get_logger().info("🚀 joy_callback triggered")
        angular_adjust = msg.axes[0]
        linear_adjust = msg.axes[1]
        right_joystick_lr = msg.axes[4]
        right_joystick_ud = msg.axes[3]

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

        forward_speed = right_joystick_ud * self.angular_speed
        turn_speed = -right_joystick_lr * self.linear_speed

        left_speed = forward_speed + turn_speed
        right_speed = forward_speed - turn_speed

        self.get_logger().info("-----------------------------------------------------------")
        self.get_logger().info(f"Joystick Axes: {msg.axes}")
        self.get_logger().info(f"Adjusted Angular Speed: {self.angular_speed:.2f}")
        self.get_logger().info(f"Adjusted Linear Speed: {self.linear_speed:.2f}")
        self.get_logger().info(f"Left Motor Speed: {left_speed:.2f}")
        self.get_logger().info(f"Right Motor Speed: {right_speed:.2f}")
        self.get_logger().info("-----------------------------------------------------------")

        self.left_motor.set_rpm(int(-left_speed * 100))
        self.right_motor.set_rpm(int(-right_speed * 100))
        self.left_motor.run()
        self.right_motor.run()

    # ---------------- encoders ----------------
    def poll_and_print_encoders(self):
        ltick = self.left_motor.read_position_feedback_ticks()
        rtick = self.right_motor.read_position_feedback_ticks()
        lrad = (2.0 * math.pi * ltick / ENC_RESOLUTION) if ltick is not None else None
        rrad = (2.0 * math.pi * rtick / ENC_RESOLUTION) if rtick is not None else None

        self.get_logger().info(
            f"[ENC] L: ticks={ltick} rad={None if lrad is None else f'{lrad:.6f}'} | "
            f"R: ticks={rtick} rad={None if rrad is None else f'{rrad:.6f}'}"
        )

    def destroy_node(self):
        try:
            self.left_motor.stop()
            self.right_motor.stop()
        except Exception:
            pass
        try:
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
