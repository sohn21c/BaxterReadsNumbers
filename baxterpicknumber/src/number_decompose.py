#!/usr/bin/env python
import rospy
import roslib
import numpy as np
from random import random
from math import sqrt
from std_msgs.msg import String,Int32,Int32MultiArray

final_result = []
raw_result = []

def prime(a):
    judge=0
    if(a==2):
        return True
    else:
        for j in range(2,int(round(sqrt(a)))+1):
            if(a%j==0):
                judge=1
            break
        if(judge==0):
            return True
        else:
            return False

def forprint(N):
    for i in range(2,int(round(sqrt(N)))+1,1):
        if(N%i==0):
            x.append(i)
            break
    N=N/i
    if(prime(N)==False):
        decompose(N)
    else:
        x.append(int(N))
    if(len(x)==1):
        return N*i
    else:
        return x

def decompose(N):
    for i in range(2,int(round(sqrt(N)))+1,1):
        if(N%i==0):
            x.append(i)
            break
    N=N/i
    if(prime(N)==False):
        decompose(N)
    else:
        x.append(int(N))
    if(len(x)==1):
        raw_result = N*i
    else:
        raw_result = x

    t = 0 #Count the position of the number in final_result
    for i in range(len(raw_result)):
	if raw_result[i]>=10:
		temp1 = raw_result[i]%10
		temp2 = (raw_result[i]-temp1)/10
		final_result[t] = temp1
		final_result[t+1] = 10
		final_result[t+2] = temp2
		t = t+3
	if raw_result[i]<10:
		final_result[t]
		t = t+1
    return final_result
   

def publish():
    rospy.init_node('number_decompose')
    decompose_publisher = rospy.Publisher('decompose',Int32MultiArray,queue_size=10)
    data = Int32MultiArray()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

      print "Please enter a number from 1 to 100"
      N = float(raw_input())
      data = decompose(N)
      rawmsg = forprint(N)
      print (N,'=',rawmsg)

      msg = Int32MultiArray()
      msg = (data)
      decompose_publisher.publish(msg)
      rate.sleep()

if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass
