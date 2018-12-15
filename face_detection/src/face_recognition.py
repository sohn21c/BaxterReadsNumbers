#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

face_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml')

eye_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_eye.xml')

def face_detection(img_data):
    gray=cv2.cvtColor(img_data,cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    return faces,gray


def training_labels(directory):
    face=[]
    face_label=[]

    for path,subdirnames,filenames in os.walk(directory):
        for filename in filenames:
            if filename.startswith("."):
                print("Skipping system file")
                continue

            id=os.path.basename(path)
            img_path=os.path.join(path,filename)
            print("img_path:",img_path)
            print("id:",id)
            img_data=cv2.imread(img_path)
            if img_data is None:
                print("Image can not be loaded")
                continue
            faces_rect,gray_img=faceDetection(img_data)
            if len(faces_rect)!=1:
               continue 
            (x,y,w,h)=faces_rect[0]
            roi_gray=gray_img[y:y+w,x:x+h]
            face.append(roi_gray)
            face_label.append(int(id))
    return face,face_label


def rectangle(img_data,face):
    (x,y,w,h)=face
    cv2.rectangle(img_data,(x,y),(x+w,y+h),(0,0,255),thickness=5)

def text(img_data,text,x,y):
    cv2.putText(img_data,text,(x,y),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,255),6)


