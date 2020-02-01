import time

import cv2
import numpy as np

import serial
from picamera import PiCamera
from picamera.array import PiRGBArray

camera = PiCamera()
camera.resolution = (320, 240)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=(320, 240))
coord = []

time.sleep(0.1)

for frame in camera.capture_continuous(rawCapture,
                                       format="bgr",
                                       use_video_port=True):
    image = frame.array
    rawCapture.truncate(0)

    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red = np.array([0, 130, 160])  # [190,0,0]
    upper_red = np.array([5, 255, 255])  # [255,255,255]
    mask0 = cv2.inRange(img_hsv, lower_red, upper_red)
    lower_red = np.array([170, 140, 40])  # [170,50,50]
    upper_red = np.array([255, 255, 255])  # [180,255,255]
    mask1 = cv2.inRange(img_hsv, lower_red, upper_red)
    mask = mask0 + mask1

    output_img = image.copy()
    output_img[np.where(mask == 0)] = 0
    output_img[np.where(mask != 0)] = 255
    red = [0, 0, 255]

    kernel3 = np.ones((3, 3), np.uint8)
    kernel9 = np.ones((9, 9), np.uint8)
    kernel5 = np.ones((5, 5), np.uint8)
    # opening = cv2.morphologyEx(output_img, cv2.MORPH_OPEN, kernel)
    # closing = cv2.morphologyEx(output_img, cv2.MORPH_CLOSE, kernel)
    dilation = cv2.dilate(output_img, kernel9, iterations=2)
    erosion = cv2.erode(dilation, kernel3, iterations=2)

    gray = cv2.cvtColor(erosion, cv2.COLOR_RGB2GRAY)

    _, contours, _ = cv2.findContours(gray, cv2.RETR_TREE,
                                      cv2.CHAIN_APPROX_SIMPLE)

    maxarea = 0
    cx = 0
    cy = 0
    coord = []
    try:
        for cnt in contours:
            rect = cv2.boundingRect(cnt)
            area = rect[2] * rect[3]

            if area > maxarea:
                maxarea = area
                mm = cv2.moments(cnt)
                cx = int(mm['m10'] / mm['m00'])
                cy = int(mm['m01'] / mm['m00'])
    except Exception:
        pass

    coord.append([cx, cy])
    with serial.Serial('/dev/ttyACM0', 115200, timeout=2) as opened_serial:
        tocenter = cx - 160

        if tocenter < -10:
            opened_serial.write(b"0")
        elif tocenter > 10:
            opened_serial.write(b"2")
        else:
            opened_serial.write(b"1")
        opened_serial.flush()

    prev_x = 160
    prev_y = 120

    for xx, xy in coord:
        cv2.line(erosion, (prev_x, prev_y), (xx, xy), red, 1)
        prev_x = xx
        prev_y = xy

    # cv2.imshow("Frame1", image)
    # cv2.imshow("OPEN", opening)
    # cv2.imshow("CLOSE", closing)
    # cv2.imshow("ERODE", erosion)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("c"):
        coord = []
        prev_x = 0
        prev_y = 0

    if key == ord("q"):
        break
