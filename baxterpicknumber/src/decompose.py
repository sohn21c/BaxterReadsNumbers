#!/usr/bin/env python
import rospy
import roslib
import numpy as np
from random import random
from math import sqrt
from std_msgs.msg import String,Int32,Int32MultiArray

if __name__ == '__main__':
    rospy.init_node('decomposer', anonymous=True, log_level=rospy.INFO)
    pub = rospy.Publisher('/multipliers', String, latch=True, queue_size=1)

    number = int(raw_input("Enter a 2 digit number: no primes please :)"))
    multipliers = []
    answer = number
    for k in range(9):
        for i in [9,8,7,6,5,4,3,2]:
            if(answer%i == 0):
                multipliers.append(i)
                answer = answer/i
                print answer
                break
    if (answer == 1):
        print("success")
        print(multipliers)
        message = ''
        for j in range(len(multipliers)):
            message = message + str(multipliers[j])
    else:
        print("sorry")
        message = 'none'
    while not rospy.is_shutdown():
        pub.publish(message)

        


