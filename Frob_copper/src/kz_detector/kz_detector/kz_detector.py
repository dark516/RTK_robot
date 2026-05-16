#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from kz_detector.seek_asy import StableDualESPDetector


class KZDetector(Node):
    """Сопоставляет данные с датчиков ESP32 и дистанцию /dist
    в ответ /ans по шаблону:

    Датчик 1 или 2 → ans в диапазоне 1..4
    Датчик 3 или 4 → ans в диапазоне 5..8

    Формула: ans = (dist_cm // 40) + base
    где base = 1 для датчиков 1-2, base = 5 для датчиков 3-4.
    """

    def __init__(self):
        super().__init__('kz_detector')

        self.declare_parameter('hostname1', 'bunderkrivitka-12.local')
        self.declare_parameter('hostname2', 'bunderkrivitka-34.local')
        self.declare_parameter('port', 8888)
        self.declare_parameter('calibration_timeout', 10.0)
        self.declare_parameter('rediscovery_interval', 5.0)

        h1 = self.get_parameter('hostname1').value
        h2 = self.get_parameter('hostname2').value
        port = self.get_parameter('port').value
        calib_timeout = self.get_parameter('calibration_timeout').value
        rediscovery = self.get_parameter('rediscovery_interval').value

        # Текущая дистанция в см (из /dist)
        self._dist_cm: float = 0.0

        # Publisher ответа
        self._ans_pub = self.create_publisher(Int32, '/ans', 10)

        # Подписка на дистанцию
        self.create_subscription(Float32, '/dist', self._dist_callback, 10)

        # Запуск детектора ESP32
        self._detector = StableDualESPDetector(
            hostname1=h1,
            hostname2=h2,
            port=port,
            on_find=self._on_sensor_find,
            calibration_timeout=calib_timeout,
            rediscovery_interval=rediscovery,
        )
        self._detector.start()

        self.get_logger().info(
            f'KZDetector запущен — hostname1={h1}, hostname2={h2}, port={port}'
        )

    def _dist_callback(self, msg: Float32):
        self._dist_cm = float(msg.data)

    def _on_sensor_find(self, sensor_idx: int):
        """Callback от StableDualESPDetector.
        sensor_idx: 0..3 (соответствует датчикам 1..4)
        """
        sensor_number = sensor_idx + 1  # 1..4
        dist = self._dist_cm

        if sensor_number in (1, 2):
            base = 1
            max_val = 4
        else:  # 3, 4
            base = 5
            max_val = 8

        ans = min(dist // 40 + base, max_val)

        msg = Int32(data=int(ans))
        self._ans_pub.publish(msg)

        self.get_logger().info(
            f'Датчик {sensor_number} | dist={dist:.0f}см | ans={int(ans)}'
        )

    def destroy_node(self):
        if hasattr(self, '_detector'):
            self._detector.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KZDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
