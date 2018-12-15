#!/usr/bin/env python

import cv2

cap=cv2.VideoCapture(0)

count = 0
while True:
    ret,test_img=cap.read()
    if not ret :
        continue
    cv2.imwrite("frame%d.jpg" % count, test_img)  
    count += 1
    resized_img = cv2.resize(test_img, (1000, 700))
    if count == 60:
        break


cap.release()
cv2.destroyAllWindows
