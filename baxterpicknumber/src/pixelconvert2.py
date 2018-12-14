#!/usr/bin/env python

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import numpy as np
from std_msgs.msg import String, Bool
import tf.transformations as tr
from ar_track_alvar_msgs.msg import AlvarMarkers
import rospy
import tf
from baxter_fun.srv import BlockLocator, BlockLocatorRequest, BlockLocatorResponse

def quaterniontoT(translation,rotation):


    R=tr.quaternion_matrix(rotation)[:-1,:-1]
    T=np.append(R,translation,axis=1)
    T=np.append(T,np.array([[0,0,0,1]]),axis=0)
    return T

class tfcovert():
    def __init__(self):
        rospy.init_node('transformer', anonymous=True, log_level=rospy.INFO)
        self.one = False
        self.two =False
        self.found=False
        self.Tca=[np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])]
        self.Tga=[np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]])]
        # self.test=AlvarMarkers()
        self.measurement=0.076

        listener = tf.TransformListener()
        rospy.sleep(5.0)
        try:
            (self.trans,self.rot) = listener.lookupTransform('/base', '/left_hand_camera', rospy.Time(0))
            
        except Exception as e: 
        	rospy.loginfo(e)
        
        self.trans = np.array([[self.trans[0],self.trans[1],self.trans[2]]]).T
        self.Tbg=quaterniontoT(self.trans,self.rot)  
        
        rospy.sleep(1.0)
        self.pink_loc=rospy.Subscriber('/pink_block_loc',String,self.pinkloccallback)
        # self.scalar_pixel=rospy.Subscriber('/pink_block',String,self.scalrcallback)
        # self.scalrcallback()
        self.target_publisher = rospy.Publisher('square_location',String, queue_size=10)
        self.Tar=rospy.Subscriber('/ar_pose_marker',AlvarMarkers,self.tf_callback)
        while not rospy.is_shutdown():
            self.target_convert()
    

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
            #Tgatemp=self.Tga_callback(data)
            #self.Tca = np.append(self.Tca,Tcatemp,axis=0)
            #self.Tca=self.Tca[-1]
            self.Tga=self.Tga_callback(data)
            #print(self.Tga)
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
            self.Tba=np.dot(self.Tbg,self.Tga)
            # print(self.Tba)
            # self.Tbc=np.array([[0,1,0,0.633],[-1,0,0,0.166],[0,0,1,1.4],[0,0,0,1]])
            # answer = np.matmul(self.Tbc,np.array([[0.1],[0.1],[1.7],[1]]))
            # rospy.loginfo(str(answer))
        else:
            rospy.loginfo("not receving enough frames")
        
    # def scalrcallback(self,pixellen):
    #     pixelleng=float(pixellen.data)

    #     self.scalar=self.measurement/pixelleng
        # rospy.loginfo(str(self.scalar))
    # def pixel_to_real_center_approach(self,x,y,theta):
    #     x_temp=(x-640)*self.scalar
    #     y_temp=(y-360)*self.scalar
    #     real_cameraframe=np.array([[x_temp,y_temp,0,1]]).T
    #     real_baseframe=(np.dot(self.Tbc,real_cameraframe))[:-1]
    #     return real_baseframe     
    def pinkloccallback(self,data):
        message=data.data.split("&")
        self.pinkx=float(message[0])
        self.pinky=float(message[1])


    def pixel_to_real_pink_approach(self,x,y,theta):
        self.Tap=np.array([[0,-1,0,0],
                            [1,0,0,-0.105],
                            [0,0,1,0],
                            [0,0,0,1]])
        # self.Tap=np.identity(4)
        
        self.Tbp=np.dot(self.Tba,self.Tap)
        # print(self.Tba)
        #self.target_publisher.publish(str(self.Tbp))
        x_temp=(x-self.pinkx)*0.00122
        y_temp=(y-self.pinky)*0.00122

        real_pinkframe=(np.array([[x_temp,y_temp,0,1]]).T)
        real_baseframe=np.dot(self.Tbp,real_pinkframe)

        
        return real_baseframe  

    
    def target_convert(self):
        rospy.wait_for_service('blockLocator')
        find_block = rospy.ServiceProxy('blockLocator', BlockLocator)
        checknumber = 'y'
        targetnumber=raw_input("=====Enter the number you want==========")
        if len(targetnumber)>1:
            if len(targetnumber)>1:
                number = int(targetnumber)
                multipliers = []
                answer = number
                for k in range(9):
                    for i in [9,8,7,6,5,4,3,2]:
                        if(answer%i == 0):
                            multipliers.append(str(i))
                            answer = answer/i
                            print answer
                            break
                if (answer == 1):
                    print("success")
                    print(multipliers)
                    numbers = []
                    
                    for j in (multipliers):
                        self.found=False
                        while(self.found==False):
                            resp1 = find_block(j)
                            if(resp1.str=="Na"):
                                continue
                            else:
                                self.found = True
                                break
                        numbers.append(resp1)
                    print("here")
                    print (numbers)
                    for num in numbers:
                        coord_block_pixel = num.str.split("&")
                        rospy.loginfo(str(coord_block_pixel))
                        block_pixel_x=float(coord_block_pixel[0])
                        block_pixel_y=float(coord_block_pixel[1])
                        block_pixel_theta=0#float(coord_block_pixel[2])
                        real_base=self.pixel_to_real_pink_approach(block_pixel_x,block_pixel_y,block_pixel_theta)
                        real_base=np.append(real_base,np.array([[block_pixel_theta]]),axis=0)
                        rospy.loginfo(str(real_base))
            
                        self.target_publisher.publish(targetnumber+"&"+str(real_base[0][0])+"&"+str(real_base[1][0])+"&"+str(real_base[2][0])+"&"+str(real_base[3][0]))
                        continue_flag = 'n'
                        while(continue_flag == 'n'):
                            continue_flag=raw_input("=====Continue==========")
                            pass

                else:
                    print("sorry")

        else:
            # while not rospy.is_shutdown():
            self.found=False
        
            while(self.found==False):
                resp1 = find_block(targetnumber)
                if(resp1.str=="Na"):
                    continue
                else:
                    self.found = True
                    break
            # rospy.loginfo(resp1)
                # except rospy.ServiceException as exc:
            # print("Service did not process request: " + str(exc))
            coord_block_pixel = resp1.str.split("&")
            rospy.loginfo(str(coord_block_pixel))
            block_pixel_x=float(coord_block_pixel[0])
            block_pixel_y=float(coord_block_pixel[1])
            block_pixel_theta=0#float(coord_block_pixel[2])
            real_base=self.pixel_to_real_pink_approach(block_pixel_x,block_pixel_y,block_pixel_theta)
            real_base=np.append(real_base,np.array([[block_pixel_theta]]),axis=0)
            rospy.loginfo(str(real_base))
            
            self.target_publisher.publish(targetnumber+"&"+str(real_base[0][0])+"&"+str(real_base[1][0])+"&"+str(real_base[2][0])+"&"+str(real_base[3][0]))

                
        
        
def main():

    
    convert=tfcovert()


if __name__ == '__main__':
	main()
	rospy.spin()





    