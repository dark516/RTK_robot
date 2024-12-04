import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import struct
import sys

class StateSubscriber(Node):

    def __init__(self, port):
        super().__init__('state_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            '/state',
            self.state_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.serial_port = serial.Serial(port, 115200, timeout=1)  # Подключаем последовательный порт

    def state_callback(self, msg):
        state_value = msg.data
        self.get_logger().info(f'Received state: {state_value}')
        # Отправляем число в бинарном формате (4 байта, int32)
        self.serial_port.write(struct.pack('i', state_value))

def main(args=None):
    rclpy.init(args=args)
    serial_port = '/dev/ttyACM1'  # Значение по умолчанию
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    state_subscriber = StateSubscriber(serial_port)
    rclpy.spin(state_subscriber)
    state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
