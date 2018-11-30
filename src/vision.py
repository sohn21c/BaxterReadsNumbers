#!/usr/bin/python

import sys
import os
import cv2
import numpy as np
import time
import heapq
import rospy
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rospkg

from std_msgs.msg import String, Float32MultiArray

bridge = CvBridge()
new_image = 0

def main(cap):
    frame = cap
    # Capture frame-by-frame
    #ret, frame = cap.read()
    frame = cv2.flip(frame,1)
    cv2.imshow('name', frame)
    cv2.waitKey(2)
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    #get values from trackbars for tolerance of hsv values (RED)
    rhmin = cv2.getTrackbarPos('redHueMin','Filter')
    rsmin = cv2.getTrackbarPos('redSaturationMin','Filter')
    rvmin = cv2.getTrackbarPos('redValueMin','Filter')

    rhmax = cv2.getTrackbarPos('redHueMax','Filter')
    rsmax = cv2.getTrackbarPos('redSaturationMax','Filter')
    rvmax = cv2.getTrackbarPos('redValueMax','Filter')
    ############################################

    #get values from trackbars for tolerance of hsv values (YELLOW)
    yhmin = cv2.getTrackbarPos('yellowHueMin','Filter')
    ysmin = cv2.getTrackbarPos('yellowSaturationMin','Filter')
    yvmin = cv2.getTrackbarPos('yellowValueMin','Filter')

    yhmax = cv2.getTrackbarPos('yellowHueMax','Filter')
    ysmax = cv2.getTrackbarPos('yellowSaturationMax','Filter')
    yvmax = cv2.getTrackbarPos('yellowValueMax','Filter')
    ############################################

    #Take values from trackbars for filtering RED
    min_red_vales = np.array([rhmin,rsmin,rvmin])
    max_red_vales = np.array([rhmax,rsmax,rvmax])
    ############################################

    #Take values from trackbars for filtering Yellow
    min_yellow_vales = np.array([yhmin,ysmin,yvmin])
    max_yellow_vales = np.array([yhmax,ysmax,yvmax])
    ############################################

    ##create mask for only red
    maskRed = cv2.inRange(hsv, min_red_vales, max_red_vales)
    ########################################################

    ##create mask for only yellow
    maskYellow = cv2.inRange(hsv, min_yellow_vales, max_yellow_vales)
    ########################################################

    #Morph Operations, Erode red mask and dilate result
    kernel = np.ones((10,10),np.uint8)
    r_erosion = cv2.erode(maskRed,kernel,iterations = 1)#Erode
    r_dilation = cv2.dilate(r_erosion,kernel,iterations = 1)#Dilate
    #final result is dilation

    #Morph Operations, Erode red mask and dilate result
    y_erosion = cv2.erode(maskYellow,kernel,iterations = 1)#Erode
    y_dilation = cv2.dilate(y_erosion,kernel,iterations = 1)#Dilate
    #final result is dilation

    #red contours found from dilation. used to produce bounding box
    redContours = cv2.findContours(r_dilation.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

    #yellow contours found from dilation. used to produce bounding box
    yellowContours = cv2.findContours(y_dilation.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    x=-1
    y=-1
    box_bool= False
    #Draw RED bounding box
    if len(yellowContours) > 1:

    	#objects = sorted(yellowContours, key=cv2.contourArea, reverse=True)
    	#contours = objects[1]

        contours = max(yellowContours, key=cv2.contourArea)
        rect = cv2.minAreaRect(contours)
        ((x,y),radius) =cv2.minEnclosingCircle(contours)
        box = np.int0(cv2.boxPoints(rect)) 
        box_bool = True     
        number = (y_dilation[int(rect[0][1]-(rect[1][1]/2.5)):int((rect[1][1]/2.5)+rect[0][1]), int(rect[0][0]-(rect[1][0]/2.5)):int((rect[1][0]/2.5)+rect[0][0])] )
        number = cv2.flip(number, 1)
        gray = cv2.resize(255 - number, (28, 28))
        #M = cv2.getRotationMatrix2D((int(rect[1][0])/2,int(rect[1][1])/2),rect[3],1)
        #number = cv2.warpAffine(number,M,(cols,rows))            
        #check if big enough, else reject and don't draw bounding box
        if radius > 25:
            cv2.drawContours(frame,[box],0,(0,0,0),1)
            cv2.circle(frame, (int(x),int(y)), int(radius), color=(150,150,150), thickness=1, lineType=8, shift=0) 

    #Draw Yellow (green) bounding box
    # if len(yellowContours) > 0:
    #     contours = max(yellowContours, key=cv2.contourArea)
    #     rect = cv2.minAreaRect(contours)
    #     ((x,y),radius) =cv2.minEnclosingCircle(contours)
    #     box = np.int0(cv2.boxPoints(rect))                        
    #     #check if big enough, else reject and don't draw bounding box
    #     if radius > 25:
    #         cv2.drawContours(frame,[box],0,(0,255,0),1)

    #Display
    rospack = rospkg.RosPack()

# list all packages, equivalent to rospack list

# get the file path for rospy_tutorials
    path = rospack.get_path('baxter_fun')
    cv2.imshow('Red',r_dilation)
    cv2.imshow('Yellow',y_dilation)
    cv2.imshow('Tracker',frame)
    if(box_bool):
        try:
            cv2.imshow('Numbers', gray)
            pred = classo(gray)
            char_im = cv2.imread("/home/jordan/baxterws/src/baxter_fun/src/"+str(pred)+".png",cv2.IMREAD_COLOR)
            cv2.imshow('face',char_im)
            msg = cv_bridge.CvBridge().cv2_to_imgmsg(char_im, encoding="bgr8")
            pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
            pub.publish(msg)
        except  Exception as e:
            rospy.loginfo(e)
    pub = rospy.Publisher('co-ords', String)
    pub.publish(str(x)+'&'+str(y)+'&'+str(width)+'&'+str(height))
    #print((x,y))
        
 #        key = cv2.waitKey(1) & 0xFF
    # # if the `q` key was pressed, break from the loop
 #        if key == 'q':
 #            break
    
 #    #cap.release()
 #    cv2.destroyAllWindows()
    

def nothing(self):
    pass

def build():
    
    cv2.namedWindow ('Filter')
    
    #Create trakbars for filtering Red
    cv2.createTrackbar('redHueMin','Filter',101,179,nothing)
    cv2.createTrackbar('redSaturationMin','Filter',56,255,nothing)
    cv2.createTrackbar('redValueMin','Filter',56,255,nothing)

    cv2.createTrackbar('redHueMax','Filter',179,179,nothing)
    cv2.createTrackbar('redSaturationMax','Filter',255,255,nothing)
    cv2.createTrackbar('redValueMax','Filter',255,255,nothing)
    #********************************************************

    #Create trakbars for filtering Blue
    cv2.createTrackbar('blueHueMin','Filter',0,179,nothing)
    cv2.createTrackbar('blueSaturationMin','Filter',100,255,nothing)
    cv2.createTrackbar('blueValueMin','Filter',100,255,nothing)

    cv2.createTrackbar('blueHueMax','Filter',179,179,nothing)
    cv2.createTrackbar('blueSaturationMax','Filter',255,255,nothing)
    cv2.createTrackbar('blueValueMax','Filter',255,255,nothing)
    #********************************************************
    
    #Create trakbars for filtering Yellow
    cv2.createTrackbar('yellowHueMin','Filter',51,179,nothing)
    cv2.createTrackbar('yellowSaturationMin','Filter',79,255,nothing)
    cv2.createTrackbar('yellowValueMin','Filter',144,255,nothing)

    cv2.createTrackbar('yellowHueMax','Filter',107,179,nothing)
    cv2.createTrackbar('yellowSaturationMax','Filter',255,255,nothing)
    cv2.createTrackbar('yellowValueMax','Filter',255,255,nothing)
    #********************************************************
width = 0 
height = 0

def imagecb(data):
    # Convert Image message to CV image with blue-green-red color order (bgr8)
    global bridge
    global width
    global height
    global new_image
    try:
        img_original = bridge.imgmsg_to_cv2(data, "bgr8")
        new_image = img_original
        width = data.width
        height = data.height
    except CvBridgeError, e:
        print("==[CAMERA MANAGER]==", e)
        return None

def callback(data): 
    new_image = imagecb(data)
    #rospy.loginfo(type(new_image))

    

def listener():
    rospy.loginfo("im freeeeeee")
     # In ROS, nodes are uniquely named. If two nodes with the same
     # node are launched, the previous one is kicked off. The
     # anonymous=True flag means that rospy will choose a unique
     # name for our 'listener' node so that multiple listeners can
     # run simultaneously.
    rospy.init_node('listener', anonymous=True, log_level=rospy.INFO)
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
     # spin() simply keeps python from exiting until this node is stopped

def model(x,w):
    fx = x.flatten()
    a = w[0] + np.dot(fx.T,w[1:])
    return a.T

# multiclass softmaax cost
def multiclass_softmax(w,x,y):        
    # pre-compute predictions on all points
    all_evals = model(x,w)
    
    # compute softmax across data points
    a = np.log(np.sum(np.exp(all_evals),axis = 0)) 
    
    # compute cost in compact form using numpy broadcasting
    b = all_evals[y.astype(int).flatten(),np.arange(np.size(y))]
    cost = np.sum(a - b)
    
    # return average
    return cost/float(np.size(y))

def classo(X):
	weights = np.genfromtxt('/home/jordan/Desktop/Varsity/Term1/Deep learning/hw3/weights.csv',delimiter=',')
	mean = np.mean(X)
	std = np.std(X)
	X = ((X-mean)/std)
	pred = np.argmax(model(X,weights))
	pred_arr = model(X,weights)
	if (pred_arr[pred]>2):
		rospy.loginfo(pred_arr)
		rospy.loginfo(pred)
	return(pred)

def images():
	img0 = cv2.imread('0.png',0)
	img1 = cv2.imread('1.png',0)
	img2 = cv2.imread('2.png',0)
	img3 = cv2.imread('3.png',0)
	img4 = cv2.imread('4.png',0)
	img5 = cv2.imread('5.png',0)
	img6 = cv2.imread('6.png',0)
	img7 = cv2.imread('7.png',0)
	img8 = cv2.imread('8.png',0)
	img9 = cv2.imread('9.png',0)



if __name__ == '__main__':
    listener()
    build()
    while not rospy.is_shutdown():
        var = cv2.imwrite('name.jpg', new_image)
        var = cv2.imread('name.jpg', 1)
        main(var)

        #main(new_image)
