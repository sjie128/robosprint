import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial, math

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.WHEEL_SEP   = 0.20   # metres between wheels — adjust to your robot
        self.MAX_SPEED   = 255

    def cmd_cb(self, msg: Twist):
        v = msg.linear.x      # forward speed  m/s
        w = msg.angular.z     # turn rate       rad/s
        # Differential drive mixing
        left  = v - w * self.WHEEL_SEP / 2.0
        right = v + w * self.WHEEL_SEP / 2.0
        # Normalise to -255..255
        scale = self.MAX_SPEED / max(abs(left), abs(right), 1.0)
        l_pwm = int(left  * scale)
        r_pwm = int(right * scale)
        cmd = f"L:{l_pwm},R:{r_pwm}\n"
        self.ser.write(cmd.encode())

def main():
    rclpy.init()
    rclpy.spin(MotorBridge())

if __name__ == '__main__':
    main()

