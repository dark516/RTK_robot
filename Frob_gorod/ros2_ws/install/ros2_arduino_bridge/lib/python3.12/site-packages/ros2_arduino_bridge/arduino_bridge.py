from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import rclpy
from serial import Serial
from time import sleep
import rclpy
import sys

from ros2_arduino_bridge.connection import ArduinoConnection

class Arduino_bridge(Node):
    def __init__(self, connection: ArduinoConnection):
        super().__init__('arduino_bridge')

        self._connect = connection

        self.subscription = self.create_subscription(
            Int32,
            "/task/state",
            self.state_callback,
            10)
            
            #Возможно убрать еденицу subscription1
        self.subscription1 = self.create_subscription(
            Int32,
            "/camera/ans",
            self.ans_callback,
            10)
            
        #self.data_request_timer = self.create_timer(0.1, self.data)  # Запрос данных раз в 1 секунду

    def state_callback(self, msg):
        # Отправка состояния робота
        self._connect.set_state(msg.data)
        
    def ans_callback(self, msg):
        # Отправка камеры
        self._connect.send_ans(msg.data)
        
    def shutdown(self) -> None:
        self._connect.close()
        super().shutdown()


def main(args=None):
    rclpy.init(args=args)

    serial_port = '/dev/ttyACM0'  # Значение по умолчанию
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]

    arduino_bridge = Arduino_bridge(ArduinoConnection(Serial(serial_port, 115200)))
    sleep(2)  # TODO

    rclpy.spin(arduino_bridge)

    arduino_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#self.publisher1.publish(Int32(data=number_for_topic1))
#self._connect.setSpeeds(left_motor_speed, right_motor_speed)

