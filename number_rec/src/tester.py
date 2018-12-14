#!/usr/bin/python

from baxter_fun.srv import BlockLocator, BlockLocatorRequest, BlockLocatorResponse
import rospy




#rospy.wait_for_service('find_block')
find_block = rospy.ServiceProxy('blockLocator', BlockLocator)
while not rospy.is_shutdown():
	try:
		resp1 = find_block("1")
	except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))