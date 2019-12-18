import cv2
import time
import numpy as np

capture = cv2.VideoCapture(0)


lower_red = np.array([-17, 100, 100])
upper_red = np.array([3, 255, 255])

value_threshold = 50

image_width = 640
scan_width, scan_height = 400, 150
lmid = scan_width
area_width, area_height = 50, 30
roi_vertical_pos = 200
row_begin = (scan_height-area_height)//2
row_end = row_begin+area_height
pixel_cnt_threshold = 0.8*area_width*area_height
while True:
    ret, frame = capture.read()

    if not ret:
        break
    if cv2.waitKey(1) & 0xFF == 27:
        break

    frame = cv2.GaussianBlur(frame, (3, 3), 0)
    frame1 = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(frame1, cv2.HOUGH_GRADIENT, 1,
                               20, param1=30, param2=70, minRadius=0, maxRadius=100)
    if circles is not None:  # 원 검출
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            radius = i[2]

            cv2.circle(frame, center, radius, (0, 255, 0), 2)
            roi = frame[roi_vertical_pos:roi_vertical_pos+scan_height, :]
            img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            mask_red = cv2.inRange(hsv, lower_red, upper_red)
            result_red = cv2.bitwise_and(hsv, hsv, mask=mask_red)  # 빨간색 검출
            left = -1
            view = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)  # 레드 검출 체크

            for l in range(area_width, lmid):
                area = mask_red[row_begin:row_end, l - area_width:l]
                if cv2.countNonZero(area) > pixel_cnt_threshold:
                    left = l
                    break
            if left != -1:  # 빨간불이면 정지
                print("stop")
            else:
                print("???")

            if left != -1:  # 감지하는지 체크
                lsquare = cv2.rectangle(
                    view, (left - area_width, row_begin), (left, row_end), (0, 255, 0), 3)

            cv2.imshow("view", view)
    cv2.imshow('img_color', frame)


cv2.destroyAllWindows()
