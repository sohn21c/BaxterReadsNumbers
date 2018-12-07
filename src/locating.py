#!/usr/bin/env python

import rospy

from std_msgs.msg import String
from random import random


coord_block = [0,0]
coord_camera = []

def callback_block(data):
	global coord_block
	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	coord_block = data.data.split("&")

	#rospy.loginfo(coord_block)

def callback_camera(data):
	global coord_camera
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	coord_camera = data.data.split("&")
	rospy.loginfo(coord_camera)

def locating():

	# init node
	rospy.init_node('locating')
	rospy.loginfo("initializing the node")
	pub = rospy.Publisher('square-location', String)
	rate = rospy.Rate(1)

	
	# subscribe to topic that is publishing the coordinate of the block of interest
	rospy.Subscriber("co-ords", String, callback_block)
	coord_x = coord_block[0]
	coord_y = coord_block[1]
	coord_z = -0.209
	theta = 0

	# Coordinate computation


	
	"""
	# subscribe to topic publishing the position of the left arm of Baxter
	rospy.Subscriber("robot/limb/left/endpoint_state", EndpointState, callback_camera)
	"""
	
	# publish to Baxter
	while not rospy.is_shutdown():
	
		
		# Publishing world coordinate to the topic square-location
		pub.publish(str(coord_block[0])+"&"+str(coord_block[1])+"&"+str(coord_z)+"&"+str(theta))

		rospy.loginfo("x %s and y %s", coord_block[0], coord_block[1])
		
	


	

if __name__=='__main__':
    try:
        
        locating()
        

    except KeyboardInterrupt:
        print("Shutting down")