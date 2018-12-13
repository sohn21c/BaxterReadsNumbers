#!/usr/bin/env python

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import numpy as np
from std_msgs.msg import String, Bool
import tf.transformations as tr
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospy
import tf

def quaterniontoT(translation,rotation):


    R=tr.quaternion_matrix(rotation)[:-1,:-1]
    T=np.append(R,translation,axis=1)
    T=np.append(T,np.array([[0,0,0,1]]),axis=0)
    return T

class tfcovert():
    def __init__(self):
        self.one = False
        self.two =False
        self.Tca=[np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])]
        self.Tga=[np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])]
        # self.test=AlvarMarkers()
        self.measurement=0.3

        listener = tf.TransformListener()
        try:
            (self.trans,self.rot) = listener.lookupTransform('/base', '/left_hand_camera', rospy.Time(0))
            
        except Exception as e: 
        	rospy.loginfo(e)
        self.trans = np.array([[self.trans[0],self.trans[1],self.trans[2]]]).T
        self.Tbg=quaterniontoT(self.trans,self.rot)  
        self.Tar=rospy.Subscriber('/ar_pose_marker',AlvarMarkers,self.tf_callback)
        self.scalar_pixel=rospy.Subscriber('/pixel_scalar',String,self.scalrcallback)
    

    def Tga_callback(self,message):
        x = (message.markers[0]).pose.pose.position.x
        y = (message.markers[0]).pose.pose.position.y
        z = (message.markers[0]).pose.pose.position.z
        qx = (message.markers[0]).pose.pose.orientation.x
        qy = (message.markers[0]).pose.pose.orientation.y
        qz = (message.markers[0]).pose.pose.orientation.z
        qw = (message.markers[0]).pose.pose.orientation.w
        translation=np.array([[x,y,z]]).T
        rotationqua=np.array([qx,qy,qz,qw])
        Tga=quaterniontoT(translation,rotationqua)
        return Tga

    def Tca_callback(self,message):
        x = (message.markers[0]).pose.pose.position.x
        y = (message.markers[0]).pose.pose.position.y
        z = (message.markers[0]).pose.pose.position.z
        qx = (message.markers[0]).pose.pose.orientation.x
        qy = (message.markers[0]).pose.pose.orientation.y
        qz = (message.markers[0]).pose.pose.orientation.z
        qw = (message.markers[0]).pose.pose.orientation.w
        translation=np.array([[x,y,z]]).T
        rotationqua=np.array([qx,qy,qz,qw])
        Tca=quaterniontoT(translation,rotationqua)
        return Tca

    def tf_callback(self,data):
    	rospy.sleep(1)

        if (data.markers[0]).header.frame_id =="left_hand_camera":
            Tgatemp=self.Tga_callback(data)
            #self.Tca = np.append(self.Tca,Tcatemp,axis=0)
            #self.Tca=self.Tca[-1]
            self.Tga=Tgatemp
            self.two = True
        if (data.markers[0]).header.frame_id=="overhead_camera":
            Tcatemp=self.Tca_callback(data)
            #self.Tga = np.append(self.Tga,Tgatemp,axis=0)
            #self.Tga=self.Tga[-1]
            self.Tca=Tcatemp
            self.one = True
        rospy.sleep(1.0)
        if (self.one and self.two):
            self.Tbc=np.dot(np.dot(self.Tbg,self.Tga),np.linalg.inv(self.Tca))
            answer = np.matmul(self.Tbc,np.array([[0.1],[0.1],[1.7],[1]]))
            rospy.loginfo(str(answer))
        else:
            rospy.loginfo("not receving enough frames")
        
    def scalrcallback(self,pixellen):
        pixellen=pixellen[0]

        self.scalar=self.measurement/pixellen
        rospy.loginfo(str(self.scalar))

        
        
def main():

    rospy.init_node('transformer', anonymous=True, log_level=rospy.INFO)
    convert=tfcovert()


if __name__ == '__main__':
	main()
	rospy.spin()





    