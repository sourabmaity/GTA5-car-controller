'''import pyvjoy
import time
#Pythonic API, item-at-a-time

j = pyvjoy.VJoyDevice(1)
time.sleep(5)
print("rady")
#turn button number 15 on
a = 20000
val=1
j.update()

while 1:

    j.set_button(val,1)
    time.sleep(5)
    j.set_button(val,0)
    time.sleep(5)
    val+=1
    if(val==8):
        val=1
'''




import cv2
import numpy as np
import pyvjoy
j = pyvjoy.VJoyDevice(1)
def left_right(angle):
    j.set_axis(pyvjoy.HID_USAGE_X, 32768-int(16384+16384*angle))

def speed(acc):
    j.set_button(6, 0)
    j.set_axis(pyvjoy.HID_USAGE_Z,0)
    j.set_axis(pyvjoy.HID_USAGE_RZ, int(327.68*acc))

def stop():
    j.set_axis(pyvjoy.HID_USAGE_RZ, 0)
    j.set_axis(pyvjoy.HID_USAGE_X, 16384)
    j.set_button(6, 1)

def reverce(acc):
    j.set_button(6, 0)
    j.set_axis(pyvjoy.HID_USAGE_Z, int(327.68 * acc))

def control():
    cap = cv2.VideoCapture(0)
    cap.set(3, 400)
    cap.set(4,400)
    while True:
        cx = []
        cy = []
        count = 0
        _, frame = cap.read()
        frame = cv2.resize(frame,(400,400))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow('frame125', hsv)
        control_l = np.array([153,144,79], np.uint8)
        control_u = np.array([188,255,255], np.uint8)
        control = cv2.inRange(hsv, control_l, control_u)
        kernal = np.ones((5, 5,), "uint8")
        control = cv2.dilate(control, kernal)
        (contours, hierarchy) = cv2.findContours(control, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if (area > 200):
                x, y, w, h = cv2.boundingRect(contour)
                img2 = frame.copy()
                img2 = img2[(y - 10):(y + h + 10), (x - 10):(x + w + 10)]
                # cv2.imshow("d", img2)
                cx.append(int(x + w / 2))
                cy.append(int(y + h / 2))
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.circle(frame, (int(x + w / 2), int(y + h / 2)), 2, (0, 0, 255), 2)
                count +=1

        if(count<=1):
            reverce(16384)
        else:
            try:
                slope = (cy[0] - cy[1]) / (cx[0] - cx[1])
            except :
                slope = 16384

            distance = abs(cx[0] - cx[1]) - w
            if(distance<20):
                stop()
            elif distance>20 and distance<=100:
                speed(distance)
            elif distance>100:
                speed(100)
            left_right(slope)
        key = cv2.waitKey(1)
        cv2.imshow("control", frame)
        if key == 27:
            break
control()
cv2.destroyAllWindows()