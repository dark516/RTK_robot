import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker

class SignDisplayNode(Node):
    def __init__(self):
        super().__init__('sign_display_node')

        # Переменные для хранения текущего состояния
        self.current_text = None
        self.current_pose = None

        # Подписка на топик с текстом знака
        self.sign_sub = self.create_subscription(
            String,
            '/signs',
            self.sign_callback,
            10
        )

        # Подписка на позицию робота от AMCL
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        # Паблишер маркера для RViz
        self.marker_pub = self.create_publisher(
            Marker, 
            '/sign_marker', 
            10
        )

        # Таймер для постоянной публикации маркера (например, 10 Гц)
        self.timer = self.create_timer(0.1, self.publish_marker)

        self.get_logger().info('Нода запущена. Ожидание знаков в /signs и позиции в /amcl_pose...')

    def sign_callback(self, msg):
        self.current_text = msg.data
        self.get_logger().info(f'Получен новый знак: "{self.current_text}"')

    def pose_callback(self, msg):
        self.current_pose = msg

    def publish_marker(self):
        # Публикуем маркер только если у нас есть и текст, и позиция
        if self.current_text is None or self.current_pose is None:
            return

        marker = Marker()
        # Используем frame_id от AMCL (обычно это 'map')
        marker.header = self.current_pose.header
        marker.ns = "robot_signs"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Задаем позицию текста: координаты робота + смещение вверх по оси Z (например, на 1 метр)
        marker.pose.position.x = self.current_pose.pose.pose.position.x
        marker.pose.position.y = self.current_pose.pose.pose.position.y
        marker.pose.position.z = self.current_pose.pose.pose.position.z + 1.0
        
        # Ориентация для TEXT_VIEW_FACING не так важна, он сам повернется к камере, 
        # но мы передаем текущую ориентацию робота для валидности кватерниона
        marker.pose.orientation = self.current_pose.pose.pose.orientation

        # Настройки текста
        marker.text = self.current_text
        marker.scale.z = 0.4  # Высота букв
        
        # Цвет текста (Желтый, полностью непрозрачный)
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0 

        # Публикуем маркер
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = SignDisplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
