import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        
        # Получение пути к ресурсам пакета
        package_path = get_package_share_directory('image_display')
        image_dir = os.path.join(package_path, 'images')
        
        # Проверка существования директории
        if not os.path.exists(image_dir):
            self.get_logger().error(f"Images directory not found: {image_dir}")
            rclpy.shutdown()
            return
            
        # Формирование путей к изображениям
        image_files = [
            os.path.join(image_dir, '0.jpg'),
            os.path.join(image_dir, '1.jpg'),
            os.path.join(image_dir, '2.jpg')
        ]
        
        # Загрузка изображений
        self.images = []
        for img_path in image_files:
            if not os.path.exists(img_path):
                self.get_logger().error(f"Image file {img_path} not found!")
                rclpy.shutdown()
                return
                
            img = cv2.imread(img_path)
            if img is None:
                self.get_logger().error(f"Failed to load image {img_path}")
                rclpy.shutdown()
                return
                
            self.images.append(img)
            self.get_logger().info(f"Successfully loaded: {img_path}")

        # Инициализация подписчика
        self.subscription = self.create_subscription(
            Int32,
            '/camera/ans',
            self.image_callback,
            10)
            
        # Настройка OpenCV окна
        self.window_name = 'RTK Image Display'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        self.current_image = self.images[0]
        cv2.imshow(self.window_name, self.current_image)
        cv2.waitKey(1)

    def image_callback(self, msg):
        try:
            if 0 <= msg.data < len(self.images):
                self.current_image = self.images[msg.data]
                cv2.imshow(self.window_name, self.current_image)
                cv2.waitKey(1)
                #self.get_logger().info(f"Displaying image {msg.data}")
            else:
                self.get_logger().warn(f"Invalid index: {msg.data}. Must be 0-{len(self.images)-1}")
        except Exception as e:
            self.get_logger().error(f"Error in callback: {str(e)}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by keyboard interrupt")
    except Exception as e:
        node.get_logger().error(f"Fatal error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
