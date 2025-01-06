import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
import os

import cv2
from ultralytics import YOLO
import torch

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Declare parameter for showing debug windows
        self.declare_parameter('show_debug', False, ParameterDescriptor(description='Show debug windows'))
        self.show_debug = self.get_parameter('show_debug').get_parameter_value().bool_value

        self.publisher_camera_ans = self.create_publisher(Int32, '/camera/ans', 10)
        self.publisher_task_state = self.create_publisher(Int32, '/task/state', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Подключение камеры
        self.cap = cv2.VideoCapture(0)

        # Настройки для ArUco
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.arucoParams = cv2.aruco.DetectorParameters()

        # Загрузка модели YOLO
        package_path = get_package_share_directory('image_processor')
        model_path = os.path.join(package_path, 'resource', 'best1.pt')
        self.model = YOLO(model_path)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Модель запущена на устройстве: {self.device}")

        # Настройка параметров камеры
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            self.get_logger().error("Не удается подключиться к камере.")
        else:
            self.get_logger().info("Камера подключена.")

    def show_img(self, name, img):
        """Отображение изображения в окне, если show_debug=True."""
        if self.show_debug:
            width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            img = cv2.resize(img, (width//2, height//2))
            cv2.imshow(str(name), img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Не удалось получить кадр с камеры.")
            return

        # Копия кадра для обработки
        copy = frame.copy()

        # YOLO детекция

        # Уменьшаем контраст
        alpha = 0.5  # Контраст (меньше 1 для уменьшения контраста)
        beta = -10     # Смещение яркости

        # Применяем линейную трансформацию
        darker_image = cv2.convertScaleAbs(copy, alpha=alpha, beta=beta)

        results = self.model.predict(darker_image, imgsz=(224, 224), verbose=False)
        annotated_frame = results[0].plot()

        # Определение класса и отображение направления
        yolo_object_detected = False
        if results[0].boxes:
            yolo_object_detected = True
            for box in results[0].boxes:
                cls = int(box.cls[0])
                if cls == 0:
                    direction = "Лево"
                    direction_code = 1
                elif cls == 1:
                    direction = "Право"
                    direction_code = 2
                elif cls == 2:
                    direction = "Прямо"
                    direction_code = 0
                else:
                    direction = "Неизвестно"
                    direction_code = -1

                self.get_logger().info(direction)
                cv2.putText(annotated_frame, direction, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                msg_camera_ans = Int32()
                msg_camera_ans.data = direction_code
                self.publisher_camera_ans.publish(msg_camera_ans)

        self.show_img("yolov8 detection", annotated_frame)

        # ArUco детекция
        gray = cv2.cvtColor(copy, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(copy, self.arucoDict, parameters=self.arucoParams)
        aruco_detected = False
        if len(corners) > 0:
            aruco_detected = True
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(copy, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(copy, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(copy, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(copy, bottomLeft, topLeft, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(copy, (cX, cY), 4, (0, 0, 255), -1)

                cv2.putText(copy, str(markerID),
                            (topLeft[0], topLeft[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

            msg_task_state = Int32()
            msg_task_state.data = 1
            self.publisher_task_state.publish(msg_task_state)

        self.show_img("aruco detection", copy)

def main(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    rclpy.spin(camera_processor)

    camera_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
