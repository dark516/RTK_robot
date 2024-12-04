import cv2

import numpy as np

# import tkinter

import threading

import time

#cfrom PIL import Image, ImageTk

from std_msgs.msg import Int32

import rclpy

from rclpy.node import Node



stop_aruco = 10

thresh_value = 100

min_area = 2500

tomato_thresh = 0.3

eggplant_thresh = 2.2

error_mar = 60

debug_mode = 0

aruco_mode = True

flag = True



video = '/home/robot/Frob_ogorod/ros2_ws/src/image_publisher_pkg/image_publisher_pkg/video.mp4'



class ImagePublisher(Node):

    def __init__(self):

        super().__init__('image_publisher')

        self.publisher_ = self.create_publisher(Int32, '/plants', 10)

        self.death_publisher_ = self.create_publisher(Int32, '/robot_state', 10)
        # self.bridge = CvBridge()



    def pub(self, msg):

        self.publisher_.publish(Int32(data=msg))

        self.get_logger().info('Publishing video frame')

    def pub_death(self, msg):
        self.death_publisher_.publish(Int32(data=msg))
        self.get_logger().info('Stop robot')





class UI:



    def __init__(self):



        self.root = tkinter.Tk()

        self.root.title('MISIS_plantation_hack')

        self.root.geometry("800x480")



        def submit_button():

            global thresh_value, min_area, tomato_thresh, eggplant_thresh



            thresh_value = int(thresh_value_var.get())

            min_area = int(min_area_value_var.get())

            tomato_thresh = float(tomato_thresh_value_var.get())

            eggplant_thresh = float(eggplant_thresh_value_var.get())

            # print(thresh_value, min_area, tomato_thresh, eggplant_thresh)



        def debug_mode_button():

            global debug_mode



            if check_var.get() == 1:

                debug_mode = True

            else:

                debug_mode = False



        def aruco_mode_button():

            global aruco_mode



            if check_aruco_var.get() == 1:

                aruco_mode = True

            else:

                aruco_mode = False



        thresh_value_var = tkinter.DoubleVar()

        thresh_scale = tkinter.Scale(self.root, variable=thresh_value_var, from_= 1, to = 255, orient= tkinter.HORIZONTAL)

        thresh_scale.set(140)

        thresh_scale_label = tkinter.Label(self.root, text="Thresholding")

        thresh_scale.pack(anchor='ne', padx=22)

        thresh_scale_label.pack(anchor='ne', padx=40)



        min_area_value_var = tkinter.DoubleVar()

        min_area_scale = tkinter.Scale(self.root, variable=min_area_value_var, from_=0, to=10000, orient=tkinter.HORIZONTAL)

        min_area_scale.set(3500)

        min_area_scale_label = tkinter.Label(self.root, text="Минимальная площадь")

        min_area_scale.pack(anchor='ne', padx=22)

        min_area_scale_label.pack(anchor='ne', padx=10)



        tomato_thresh_value_var = tkinter.DoubleVar()

        tomato_thresh_scale = tkinter.Scale(self.root, variable=tomato_thresh_value_var, from_= 0, to=1, resolution=0.01, orient=tkinter.HORIZONTAL)

        tomato_thresh_scale.set(0.3)

        tomato_thresh_scale_label = tkinter.Label(self.root, text="Коэффициент перца")

        tomato_thresh_scale.pack(anchor='ne', padx=22)

        tomato_thresh_scale_label.pack(anchor='ne', padx=15)



        eggplant_thresh_value_var = tkinter.DoubleVar()

        eggplant_thresh_scale = tkinter.Scale(self.root, variable=eggplant_thresh_value_var, from_=1, to=5, resolution=0.01, orient=tkinter.HORIZONTAL)

        eggplant_thresh_scale.set(2.2)

        eggplant_thresh_scale_label = tkinter.Label(self.root, text="Коэффициент лимона")

        eggplant_thresh_scale.pack(anchor='ne', padx=20)

        eggplant_thresh_scale_label.pack(anchor='ne', padx=2)



        b1 = tkinter.Button(self.root, text="Submit", command=submit_button, bg="green")

        b1.pack(anchor='ne', padx=50, pady=5)



        check_var = tkinter.IntVar()

        checkbox = tkinter.Checkbutton(self.root, text='Режим отладки', command=debug_mode_button, onvalue=1, offvalue=0, variable=check_var)

        checkbox.pack(anchor='ne', padx=20, pady=35)



        label_widget = tkinter.Label(self.root)

        self.label = label_widget

        label_widget.pack()



        check_aruco_var = tkinter.IntVar()

        checkbox_2 = tkinter.Checkbutton(self.root, text='Искать Aruco?', command=aruco_mode_button, onvalue=1,

                                         offvalue=0, variable=check_aruco_var)

        checkbox_2.pack(anchor='ne', padx=26)





class Contour:



    def __init__(self, c):

        self.contour = c

        M = cv2.moments(c)

        if M['m00'] != 0:

            self.center_coord =(int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))

        else:

            self.center_coord = (0, 0)

        self.area = cv2.contourArea(c)

        self.type = None

        self.mean_color = None



    def get_type(self, color):

        # print([round(i, 2) for i in color])



        lemon_multiplier = 1.8

        pepper_thresh = 0.2

        pear_thresh = 0.4



        self.mean_color = color





        if (max(color) == color[0]) and abs(color[1]/color[2] - 1) < pepper_thresh and color[1]<100:

            self.type = 'Pepper'

        elif color[0] > color[2]*lemon_multiplier and color[1] > color[2]*lemon_multiplier and color[0]>120:
            self.type = 'Lemon'





class Frame:



    def __init__(self, frame):

        self.frame = self.to_RGB(frame)

        self.original_frame = frame.copy()

        self.filtered = None

        self.threshed = None

        self.filled = None

        self.largest_contour = None



    def color_filtering(self, given_frame, x_range, y_range, marker_corners):

        self.filled = np.full(self.original_frame.shape, 255, dtype=np.uint8)

        contours, hierarchy = cv2.findContours(image=given_frame, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)

        max_color = [0, 0, 0]

        max_con = None

        for cn in contours:

            cur_contour = Contour(cn)

            if cur_contour.area > 2500:

                mask = np.zeros(given_frame.shape[:2], np.uint8)

                cv2.drawContours(mask, cur_contour.contour, -1, 255, -1)

                cur_contour.get_type(cv2.mean(self.frame, mask=mask))

                if sum(cur_contour.mean_color) > sum(max_color):

                    max_color = cur_contour.mean_color

                    max_con = cur_contour

        if max_con is not None:

            if marker_corners and not (x_range[0] - error_mar <= max_con.center_coord[0] <= x_range[1] + error_mar and y_range[1] - error_mar <= max_con.center_coord[1] <= y_range[0] + error_mar) or not marker_corners:

                cv2.fillPoly(self.filled, pts=[max_con.contour], color=max_con.mean_color)

                self.largest_contour = max_con

                # if marker_corners and not (

                #         x_range[0] - error_mar <= cur_contour.center_coord[0] <= x_range[

                #     1] + error_mar and y_range[0] - error_mar <= cur_contour.center_coord[1] <= y_range[

                #             1] + error_mar) or not marker_corners:

                #     max_contour_area = cur_contour.area

                #     self.largest_contour = cur_contour



    def bi_filter(self, given_frame):

        self.filtered = cv2.bilateralFilter(given_frame, 15, 150, 150)



    def thresh(self, given_frame, thresh_value):

        self.threshed = cv2.threshold(given_frame, thresh_value, 255, cv2.THRESH_BINARY_INV)[1]



    def to_Gray(self, frame_to_change):

        return cv2.cvtColor(frame_to_change, cv2.COLOR_RGB2GRAY)



    def to_RGB(self, frame_to_change):

        return cv2.cvtColor(frame_to_change, cv2.COLOR_BGR2RGB)



    def to_BGR(self, frame_to_change):

        return cv2.cvtColor(frame_to_change, cv2.COLOR_RGB2BGR)





class Aruco:



    def __init__(self):

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)



    def detect(self, frame_to_detect):

        marker_corners, marker_IDs, reject = self.detector.detectMarkers(frame_to_detect)

        return marker_corners, marker_IDs





def main(args=None):



    def wait_after_sanding(sec):

        global flag

        flag = False

        # print('Я жду')

        time.sleep(sec)

        image_publisher.pub(2)

        flag = True



    rclpy.init(args=args)

    image_publisher = ImagePublisher()



    cap = cv2.VideoCapture(0)



    width = cap.get(3)  # float `width`

    height = cap.get(4)  # float `height`



    detector = Aruco()

    # window = UI()



    while True:

        ret, frame = cap.read()

        cur_frame = Frame(frame)



        cur_frame.bi_filter(cur_frame.frame)

        cur_frame.thresh(cur_frame.filtered, thresh_value)



        marker_corners, marker_IDs = detector.detect(cur_frame.frame)



        x_range = [0, 0]

        y_range = [0, 0]



        if marker_corners:

            for ids, corners in zip(marker_IDs, marker_corners):

                corners = corners.reshape(4, 2)

                corners = corners.astype(int)

                top_right = corners[0].ravel()

                top_left = corners[1].ravel()

                bottom_right = corners[2].ravel()

                bottom_left = corners[3].ravel()

                if aruco_mode:

                    print(f"Обнаружен Aruco-маркер с номером {str(*ids)}")

                    cv2.polylines(cur_frame.original_frame, [corners], True, (0, 255, 255), 2, cv2.LINE_AA)

                    cv2.putText(cur_frame.original_frame, str(*ids),

                                bottom_left, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2, 2)

                    if int(*ids) == stop_aruco:

                        time.sleep(1.5)
                        image_publisher.pub_death(1)



                x_range = [min(top_left[0], bottom_left[0]), max(top_right[0], bottom_right[0])]

                x_range = [min(x_range), max(x_range)]

                y_range = [max(top_left[1], top_right[1]), min(bottom_left[1], bottom_right[1])]

                y_range = [min(y_range), max(y_range)]



        cur_frame.color_filtering(cur_frame.to_Gray(cur_frame.threshed), x_range, y_range, marker_corners)



        if cur_frame.largest_contour is not None:

            if width*0.1 < cur_frame.largest_contour.center_coord[0] < width*0.9:

                name = str(cur_frame.largest_contour.type)

                if flag:

                    if name == 'Lemon':

                        print(f"Был обнаружен {name}")
                        time.sleep(0.2)

                        image_publisher.pub(1)

                        t1 = threading.Thread(target=wait_after_sanding, args=(2.5,))

                        t1.start()



                    elif name == 'Pepper':

                        print(f"Был обнаружен {name}")

                        time.sleep(0.2)

                        image_publisher.pub(1)

                        t1 = threading.Thread(target=wait_after_sanding, args=(2.5,))

                        t1.start()



                    elif name == 'Pear':

                        print(f"Был обнаружен {name}")

                        # image_publisher.pub(1)

                cv2.circle(cur_frame.original_frame, cur_frame.largest_contour.center_coord, 7, (0, 255, 0), -1)

                cv2.putText(cur_frame.original_frame, str(cur_frame.largest_contour.type), cur_frame.largest_contour.center_coord, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, 2)



        # if not debug_mode:

        #    tmp_img = Image.fromarray(cur_frame.to_RGB(cur_frame.original_frame))

        # else:

        #    tmp_img = Image.fromarray(cur_frame.filled)

        #photo_tmp_img = ImageTk.PhotoImage(image=tmp_img)

        #window.label.photo_image = photo_tmp_img

        #window.label.place(x=0, y=0)

        #window.label.configure(image=photo_tmp_img)

        #window.root.update()

        #k = cv2.waitKey(30) & 0xff

        #if k == 27:

        #    break





if __name__ == '__main__':

    main()
