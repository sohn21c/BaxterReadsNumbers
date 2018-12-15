#!/usr/bin/env python

import cv2
import os
import numpy as np
import face_recognition as face_rec




face_detec,face_label=face_rec.training_labels('/home/lukai/catkin_ws/src/face_detection/image')
#face_recognizer=fr.train_classifier(faces,faceID)
face_recognizer=cv2.face.createLBPHFaceRecognizer()
face_recognizer.train(face_detec,np.array(face_label))
#uncomment this line if you want to save the training data
#face_recognizer.load('trainingData.yml')
name={0:"LUKAI",1:"MARK"}

cap=cv2.VideoCapture(0)

while True:
    ret,img_data=cap.read()
    faces_detected,gray=face_rec.face_detection(img_data)
    for (x,y,w,h) in faces_detected:
      cv2.rectangle(img_data,(x,y),(x+w,y+h),(255,0,0),thickness=3)

    for face in faces_detected:
        (x,y,w,h)=face
        gray_area=gray[y:y+w, x:x+h]
        label,confidence=face_recognizer.predict(gray_area)
        print("confidence:",confidence)
        print("label:",label)
        face_rec.rectangle(img_data,face)
        predicted_name=name[label]
        if confidence < 39:
	   print(confidence)
           face_rec.text(img_data,predicted_name,x,y)

    resized_img = cv2.resize(img_data, (1000, 700))
    cv2.imshow('face_recognition',resized_img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
	break   

cap.release()
cv2.destroyAllWindows
