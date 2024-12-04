
from ultralytics import YOLO

import cv2

import numpy as np

# import tkinter



threshold_value = 0.5

aruco_mode = True

min_area = 2500



oil_dict = {}

video = 'C:/Users/georg/Pictures/Camera Roll/WIN_20241128_18_01_30_Pro.mp4'





class Aruco:



    def __init__(self):

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)

        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)



    def detect(self, frame_to_detect):

        marker_corners, marker_IDs, reject = self.detector.detectMarkers(frame_to_detect)

        return marker_corners, marker_IDs





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

        self.greatest_color = None

        self.mean_color_mean = None



    def get_type(self, color):

        self.mean_color = [round(color[0], 2), round(color[1], 2), round(color[2], 2)]

        self.mean_color_mean = int((self.mean_color[0] + self.mean_color[1] + self.mean_color[2]) / 3)

        self.greatest_color = max(self.mean_color)





class Frame:



    def __init__(self, frame):

        self.frame = self.to_RGB(frame)

        self.original_frame = self.frame.copy()

        self.filtered = None

        self.threshed = None

        self.filled = None

        self.largest_contour = None

        self.closing = None

        self.erosion = None



    def color_filtering(self, given_frame, y_range):

        self.filled = np.full(self.original_frame.shape, 255, dtype=np.uint8)

        contours, hierarchy = cv2.findContours(image=given_frame, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

        for cn in contours:

            cur_contour = Contour(cn)

            if cur_contour.area > min_area:

                mask = np.zeros(given_frame.shape[:2], np.uint8)

                cv2.drawContours(mask, cur_contour.contour, -1, 255, -1)

                cur_contour.get_type(cv2.mean(self.frame, mask=mask))

                epsilon_1 = 0.025 * cv2.arcLength(cn, True)

                approx_edges = cv2.approxPolyDP(cn, epsilon_1, True)

                epsilon_2 = 0.015 * cv2.arcLength(cn, True)

                approx_area = cv2.approxPolyDP(cn, epsilon_2, True)

                cv2.fillPoly(self.filled, pts=[approx_edges], color=cur_contour.mean_color)

                cv2.fillPoly(self.filled, pts=[approx_edges], color=cur_contour.mean_color)

                x, y, w, h = cv2.boundingRect(cn)

                cv2.fillPoly(self.filled, pts=[cur_contour.contour], color=cur_contour.mean_color)

                cv2.putText(self.filled, str(cur_contour.mean_color), cur_contour.center_coord,

                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, 1)



                if cv2.contourArea(approx_area) * 1.25 >= (w * h) and cv2.contourArea(approx_area) < 5000 and (

                        cur_contour.mean_color[0] < 40 and cur_contour.mean_color[1] < 70 and cur_contour.mean_color[

                    2] < 50) and 100 < cur_contour.center_coord[1]:

                    print(f"Обнаружена протечка!")

                    cv2.rectangle(self.original_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)



    def bi_filter(self, given_frame):

        self.filtered = cv2.bilateralFilter(given_frame, 15, 150, 150)



    def thresh(self, given_frame, thresh_value):

        _, self.threshed = cv2.threshold(given_frame, thresh_value, 255, cv2.THRESH_BINARY_INV)



    def to_Gray(self, frame_to_change):

        return cv2.cvtColor(frame_to_change, cv2.COLOR_RGB2GRAY)



    def to_RGB(self, frame_to_change):

        return cv2.cvtColor(frame_to_change, cv2.COLOR_BGR2RGB)



    def to_BGR(self, frame_to_change):

        return cv2.cvtColor(frame_to_change, cv2.COLOR_RGB2BGR)



    def to_close(self, frame_to_change, kernel, iterations):

        self.closing = cv2.morphologyEx(frame_to_change, cv2.MORPH_CLOSE, kernel, iterations=iterations)



    def to_erode(self, frame_to_change, kernel, iterations):

        self.erosion = cv2.erode(frame_to_change, kernel, iterations=iterations)





class UI:



    def __init__(self):

        self.root = tkinter.Tk()

        self.root.title('MISIS_oil_hack')

        self.root.geometry("800x520")

        self.label = tkinter.Label(self.root)

        self.label.place(x=0, y=0)



        def submit_button():

            global threshold_value, min_area



            threshold_value = float(thresh_value_var.get())

            min_area = int(min_area_var.get())



        def aruco_mode_button():

            global aruco_mode



            if check_aruco_var.get() == 1:

                aruco_mode = True

            else:

                aruco_mode = False



        thresh_value_var = tkinter.DoubleVar()

        thresh_scale = tkinter.Scale(self.root, variable=thresh_value_var, from_=0.1, to=0.99, resolution=0.01, orient=tkinter.HORIZONTAL)

        thresh_scale.set(0.5)

        thresh_scale_label = tkinter.Label(self.root, text="Probability")

        thresh_scale.pack(anchor='ne', padx=22)

        thresh_scale_label.pack(anchor='ne', padx=40)



        min_area_var = tkinter.IntVar()

        min_area_scale = tkinter.Scale(self.root, variable=min_area_var, from_=2500, to=50000, orient=tkinter.HORIZONTAL)

        min_area_scale.set(3000)

        min_area_label = tkinter.Label(self.root, text="Min area")

        min_area_scale.pack(anchor='ne', padx=22)

        min_area_label.pack(anchor='ne', padx=40)



        b1 = tkinter.Button(self.root, text="Submit", command=submit_button, bg="green")

        b1.pack(anchor='ne', padx=50, pady=20)



        check_aruco_var = tkinter.IntVar()

        checkbox_2 = tkinter.Checkbutton(self.root, text='Искать Aruco?', command=aruco_mode_button, onvalue=1,

                                       offvalue=0, variable=check_aruco_var)

        checkbox_2.pack(anchor='ne', padx=26)







def main():

    global oil_dict



    cap = cv2.VideoCapture(0)



    detector = Aruco()

    qcd = cv2.QRCodeDetector()

    # window = UI()



    model_path = 'best 4.pt'

    # Load a model

    model = YOLO(model_path)  # load a custom model



    while True:

        ret, frame = cap.read()

        cur_frame = Frame(frame)



        marker_corners, marker_IDs = detector.detect(cur_frame.frame)

        cir_list = []



        cur_frame.bi_filter(cur_frame.frame)

        cur_frame.thresh(cur_frame.to_Gray(cur_frame.filtered), 60)

        cur_frame.color_filtering(cur_frame.threshed, 0)



        y_range = [0, 0]

        if marker_corners:



            results = model.predict(cur_frame.to_BGR(cur_frame.frame), imgsz=(224, 224), verbose=False)[0]

            for result in results.boxes.data.tolist():

                x1, y1, x2, y2, score, class_id = result

                if not aruco_mode or (aruco_mode and marker_corners):

                    if score > threshold_value:

                        cir_list.append([x1, y1, x2, y2, score, class_id, False])



            if len(cir_list) > 0:



                for ids, corners in zip(marker_IDs, marker_corners):

                    corners = corners.reshape(4, 2)

                    corners = corners.astype(int)

                    top_right = corners[0].ravel()

                    top_left = corners[1].ravel()

                    bottom_right = corners[2].ravel()

                    bottom_left = corners[3].ravel()

                    center = (int((top_right[0] + top_left[0] + bottom_right[0] + bottom_left[0]) / 4),

                              int((top_right[1] + top_left[1] + bottom_right[1] + bottom_left[1]) / 4))

                    x_range = [min(top_left[0], bottom_left[0]), max(top_right[0], bottom_right[0])]

                    x_range = [min(x_range), max(x_range)]

                    y_range = [max(top_left[1], top_right[1]), min(bottom_left[1], bottom_right[1])]

                    y_range = [min(y_range), max(y_range)]



                    if aruco_mode:

                        cv2.polylines(cur_frame.original_frame, [corners], True, (0, 255, 255), 2, cv2.LINE_AA)

                        cv2.putText(cur_frame.original_frame, str(*ids),

                                                center, cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3,

                                                cv2.LINE_AA)

                        closest = [1000000000000000000000]

                        for i in cir_list:



                            cur_dist = (((i[0] + i[2])/2 - center[0])**2 + ((i[1] + i[3])/2 - center[1])**2)**0.5

                            if closest[0] > cur_dist and min(x_range) <= int((i[0] + i[2]) / 2) <= max(x_range):

                                closest = [cur_dist, i]

                                i[-1] = True



                        if len(closest)>1:

                            b_x1, b_y1, b_x2, b_y2, b_score, b_class_id = closest[1][0], closest[1][1],closest[1][2],closest[1][3],closest[1][4],closest[1][5]

                            if not aruco_mode or (aruco_mode and marker_corners):

                                if (b_x1 - b_x2) * (b_y1 - b_y2) > min_area:

                                    name = str(results.names[int(b_class_id)].upper())

                                    cv2.rectangle(cur_frame.original_frame, (int(b_x1), int(b_y1)), (int(b_x2), int(b_y2)),

                                                  (0, 255, 0), 4)

                                    cv2.putText(cur_frame.original_frame, (name + ' : ' + str(round(b_score, 2))),

                                                (int(b_x1 - 25), int(b_y1 + 10)), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 255, 0), 3,

                                                cv2.LINE_AA)

                                    if str(*ids) not in oil_dict.keys():

                                        oil_dict[str(*ids)] = 0
                                        if str(*ids) == '15':
                                            print('Я вижу корову')

                                    # print(f"Aruco {str(*ids)} is colored {name}")

                                    if name == 'RED':

                                        oil_dict[str(*ids)] -= 1

                                        print(oil_dict)

                                    elif name == 'GREEN':

                                        oil_dict[str(*ids)] += 1

                                        print(oil_dict)



        else:

            ret_qr, decoded_info, points, _ = qcd.detectAndDecodeMulti(cv2.resize(cur_frame.frame, None, fx=2.5, fy=2.5))

            if ret_qr:

                for s, p in zip(decoded_info, points):

                    cur_frame.original_frame = cv2.resize(cur_frame.frame, None, fx=2.5, fy=2.5)

                    if s:

                        print(f"Обнаружен QR-код '{s}'")

                        color = (0, 255, 0)

                        cur_frame.original_frame = cv2.polylines(cur_frame.original_frame, [p.astype(int)], True, color, 8)



                    else:

                        color = (0, 0, 255)

                        cur_frame.original_frame = cv2.polylines(cur_frame.original_frame, [p.astype(int)], True, color, 8)

        # tmp_img = Image.fromarray(cur_frame.original_frame)

        #

        # photo_tmp_img = ImageTk.PhotoImage(image=tmp_img)

        # window.label.photo_image = photo_tmp_img

        # window.label.place(x=0, y=0)

        # window.label.configure(image=photo_tmp_img)

        # window.root.update()

        # k = cv2.waitKey(30) & 0xff

        # cv2.imshow("RESULT", cur_frame.original_frame)

        # if k == 27:

        #    break





if __name__ == '__main__':

    main()





# {'1': 16, '2': -5, '3': 11, '9': -17, '4': -13, '8': -8, '10': 17, '5': -10, '11': -10, '12': 17, '15': 7, '7': 18}т
