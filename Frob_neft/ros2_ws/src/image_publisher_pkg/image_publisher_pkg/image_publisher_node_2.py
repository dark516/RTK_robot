import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import time

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/ans_image', 10)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.aruco_numbers = {"8" : "1", "2" : "1", "9" : "2", "3" : "2", "5" : "3", "4" : "3", "7":"4", "1":"4", "16" : "5", "12" : "5", "10" : "6", "11" : "6"}
        self.stop_list = []
        self.h = 600
        self.w = 800
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)   

    def pub(self):

        def zoom_at(img, x, y, r):
            closing = []
            result = 0
            if y-(r*0.85) >= 0 and y+(r*0.95)<= self.h  and x-(r*0.85)>= 0 and x+(r*0.85)<= self.w :
                mask = np.zeros(img.shape[:2], dtype="uint8")
                cv2.circle(mask, (x, y), r, 255, -1)
                masked = cv2.bitwise_and(img, img, mask=mask)
                crop_img_orig = masked[int(y-(r*0.85)):int(y+r*0.15), int(x-(r*0.85)):int(x+(r*0.85))]
                crop_img_orig[np.where((crop_img_orig==[0,0,0]).all(axis=2))] = [255,255,255]
                brightness = 1
                contrast = 1
                crop_img = cv2.addWeighted(crop_img_orig, contrast, np.zeros(crop_img_orig.shape, crop_img_orig.dtype), 0, brightness)
                gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)

                # threshold
                crop_img = cv2.threshold(gray, 40, 255, cv2.THRESH_BINARY)[1]

                kernel1 = np.ones((5, 5), np.uint8)
                closing = cv2.morphologyEx(crop_img, cv2.MORPH_CLOSE, kernel1, iterations=1)
                closing = cv2.bitwise_not(closing)

                contours, hierarchy = cv2.findContours(image=closing, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)

                # draw contours on the original image
                image_copy = crop_img_orig.copy()
                if len(contours) > 0:
                    M = cv2.moments(contours[0])
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    # cv2.drawContours(image_copy, contours[0], -1, (0, 255, 0), 2)
                    # cv2.circle(image_copy, (cx, cy), 7, (0, 0, 255), -1)
                    # cv2.putText(image_copy, "center", (cx - 20, cy - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                    if cx < image_copy.shape[1] // 2:
                        result = 'green'
                    else:
                        result = 'red'

                closing = image_copy

            return closing, result


        aruco_dict = {}
        while rclpy.ok():
            _, tmp_frame = self.cap.read()
            original_frame = tmp_frame.copy()

            marker_corners, marker_IDs, reject = self.detector.detectMarkers(tmp_frame)
            if marker_corners:
                # marker_corners, marker_IDs, reject = detector.detectMarkers(tmp_frame)

                gray = cv2.cvtColor(tmp_frame, cv2.COLOR_BGR2GRAY)
                kernel = np.ones((6, 6), np.uint8)
                invert = cv2.bitwise_not(gray)
                res = cv2.dilate(invert, kernel, iterations=3)
                blur = cv2.GaussianBlur(res, (5, 5), 0)
                circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, 1, minDist=130, param1=120, param2=40, minRadius=50,
                                   maxRadius=160)
                found_circles = []

                if circles is not None:
                    circles = np.round(circles[0, :]).astype("int")
                    counter = 0
                    for (x, y, r) in circles:
                        cv2.circle(tmp_frame, (x, y), r, (255, 0, 0), 4)
                        cv2.rectangle(tmp_frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                        zoomed_frame, prediction = zoom_at(original_frame, x, y, r)
                        if len(zoomed_frame) != 0:
                        # cv2.imshow('None approximation', zoomed_frame)
                            counter += 1

                            found_circles.append([x, y, prediction])
                for ids, corners in zip(marker_IDs, marker_corners):
                    closest_circ_dist = -1
                    closest_circ_value = -1
                    used_circle = []
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_right = corners[0].ravel()
                    top_left = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    bottom_left = corners[3].ravel()
                    center = (int((top_right[0] + top_left[0] + bottom_right[0] + bottom_left[0]) / 4), int((top_right[1] + top_left[1] + bottom_right[1] + bottom_left[1]) / 4))
                    for cur_circ in found_circles:
                        cur_dist = ((center[0] - cur_circ[0]) ** 2 + (center[1] - cur_circ[1]) ** 2) ** 0.5
                        if closest_circ_dist == -1 or cur_dist <= closest_circ_dist:
                            closest_circ_value = cur_circ[2]
                            closest_circ_dist = cur_dist
                            used_circle = cur_circ

                    if len(used_circle) != 0:
                        found_circles.remove(used_circle)

                    if closest_circ_dist != -1:
                        if str(*ids) in aruco_dict.keys():
                            if closest_circ_value == 'green':
                                aruco_dict[str(*ids)] = aruco_dict[str(*ids)] + 0.2
                                if aruco_dict[str(*ids)] > 1:
                                    aruco_dict[str(*ids)] = 1
                            else:
                                aruco_dict[str(*ids)] = aruco_dict[str(*ids)] - 0.2
                                if aruco_dict[str(*ids)] < -1:
                                    aruco_dict[str(*ids)] = -1
                        else:
                            aruco_dict[str(*ids)] = 0
                    if str(*ids) in aruco_dict.keys() and str(*ids) not in self.stop_list:
                        if aruco_dict[str(*ids)] < -0.1:
                            cv2.polylines(tmp_frame, [corners.astype(np.int32)], True, (0, 0, 255), 2, cv2.LINE_AA)
                            cv2.circle(tmp_frame, center, 10, (0, 0, 255), 3)
                            cv2.putText(tmp_frame, str('oil pump N.' + str(*ids)), (5, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 0, 0), 2, cv2.LINE_AA)
                            print("Red monometer detected on an oil pump number " + str(*ids))
                            # ros_image = self.bridge.cv2_to_imgmsg(tmp_frame, "bgr8")
                            # self.publisher_.publish(ros_image)
                            # self.get_logger().info('Publishing video frame')
                            self.stop_list.append(str(*ids))

                            # print(aruco_dict)
                            print(self.stop_list)
                        elif aruco_dict[str(*ids)] > 0.1:
                            cv2.polylines(tmp_frame, [corners.astype(np.int32)], True, (0, 255, 0), 2, cv2.LINE_AA)
                            cv2.circle(tmp_frame, center, 10, (0, 255, 0), 3)
                            cv2.putText(tmp_frame, str('oil pump N.' + str(*ids)), (5, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 0, 0), 2, cv2.LINE_AA)
                            print("Green monometer detected on an oil pump number " + str(*ids))
                            # ros_image = self.bridge.cv2_to_imgmsg(tmp_frame, "bgr8")
                            # self.publisher_.publish(ros_image)
                            # self.get_logger().info('Publishing video frame')
                            self.stop_list.append(str(*ids))

                            # print(aruco_dict)
                            print(self.stop_list)
            frame = tmp_frame

            # ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")  
            # self.publisher_.publish(ros_image)
            # self.get_logger().info('Publishing video frame')


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
