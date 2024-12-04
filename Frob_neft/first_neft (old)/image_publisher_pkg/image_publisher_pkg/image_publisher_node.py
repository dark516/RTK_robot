import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/ans_image', 10)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.aruco_dict = {"1" : "1", "3" : "2", "9" : "3", "7" : "4", "10" : "6", "8" : "8"}

    def pub(self):
        error_mar = 30
        stations_list = []
        while rclpy.ok():

            aruco_detected = -1

            _ , img = self.cap.read()
            marker_corners, marker_IDs, reject = self.detector.detectMarkers(img)

            brightness = 0.5
            # Adjusts the contrast by scaling the pixel values by 2.3
            contrast = 0.5
            enhanced_img = cv2.addWeighted(img, contrast, np.zeros(img.shape, img.dtype), 0, brightness)
            # hsv = cv2.cvtColor(enhanced_img, cv2.COLOR_BGR2HSV)

            # lower_mask = [0, 50, 50]
            # upper_mask = [10, 255, 255]
            # lower_mask_arr = np.array(lower_mask)
            # upper_mask_arr = np.array(upper_mask)

            # mask = cv2.inRange(hsv, lower_mask_arr, upper_mask_arr)
            # masked = cv2.bitwise_and(enhanced_img, enhanced_img, mask=mask)
            # red_contours, red_h = cv2.findContours(cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            gray = cv2.cvtColor(enhanced_img, cv2.COLOR_BGR2GRAY)

            # threshold
            thresh = cv2.threshold(gray, 21, 255, cv2.THRESH_BINARY)[1]

            # define the kernel
            kernel = np.ones((6, 6), np.uint8)

            # opening the image
            closing = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=3)

            kernel2 = np.ones((15, 15), np.uint8)
            erosion = cv2.erode(closing, kernel, iterations=1)
            closing = cv2.bitwise_not(erosion)

            # get largest contour and draw on copy of input
            result = enhanced_img.copy()
            contours, h = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if marker_corners:
                for ids, corners in zip(marker_IDs, marker_corners):
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()
                    w = ((bottom_left[0] - bottom_right[0]) ** 2 + (bottom_left[1] - bottom_right[1]) ** 2) ** 0.5
                    h = ((top_left[0] - bottom_left[0]) ** 2 + (top_left[1] - bottom_left[1]) ** 2) ** 0.5
                    cv2.polylines(img, [corners], True, (0, 255, 255), 2, cv2.LINE_AA)
                    aruco_detected = ids[0]
                    x_range = [min(top_left[0], bottom_left[0]), max(top_right[0], bottom_right[0])]
                    x_range = [min(x_range), max(x_range)]
                    y_range = [max(top_left[1], top_right[1]), min(bottom_left[1], bottom_right[1])]
                    y_range = [min(y_range), max(y_range)]
                    print('I see aruco!')
                    # if aruco_detected not in stations_list:
                        # stations_list.append(aruco_detected)
                        # ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")
                        # Публикация кадра в топик
                        # self.publisher_.publish(ros_image)
                        # self.get_logger().info('Publishing video frame')

            # big_contour = max(contours, key=cv2.contourArea)
            for cnt in contours:
                x1, y1 = cnt[0][0]
                approx = cv2.approxPolyDP(cnt, 0.017 * cv2.arcLength(cnt, True), True)
                # print(approx)
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(cnt)
                    ratio = float(w) / h
                    if aruco_detected != -1:
                        # print(x_range, y_range, (x,y))
                        if not (x_range[0]-error_mar <= x <= x_range[1]+error_mar and y_range[0]-error_mar <= y <= y_range[1]+error_mar):
                            # print(ratio, cv2.contourArea(cnt), approx)
                            if cv2.contourArea(cnt) > 2000:
                                if (ratio >= 0.3 and ratio <= 3):
                                    if aruco_detected not in stations_list:
                                        if y1 > min(y_range):
                                            result = cv2.drawContours(img, [cnt], -1, (0, 255, 0), 3)
                                            cv2.putText(img, 'Leakage', (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                                            print (f'detected leakage at {self.aruco_dict[str(aruco_detected)]} oil station')
                                            cv2.putText(img, str("Leakage detected on oil pump N.") + self.aruco_dict[str(aruco_detected)], (5, 20), cv2.FONT_HERSHEY_SIMPLEX , 0.8, (255, 0, 0), 2, cv2.LINE_AA)
                                            stations_list.append(aruco_detected)

                                            ros_image = self.bridge.cv2_to_imgmsg(img, "bgr8")
                                            # Публикация кадра в топик
                                            self.publisher_.publish(ros_image)
                                            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    try:
        image_publisher.pub()  # Запуск основного цикла публикации кадров
    except KeyboardInterrupt:
        pass

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
