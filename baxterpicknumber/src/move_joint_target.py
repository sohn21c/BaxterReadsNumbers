#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
# from moveit_msgs.msg import GetPositionFK
import geometry_msgs.msg
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import numpy as np
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import baxter_interface
from baxter_interface import Gripper ,CHECK_VERSION
from sensor_msgs.msg import Range
import tf.transformations as tr
from math import sqrt
from modern_robotics import IKinBody

def converttoSE3(x,y,z,theta):
    SE3=np.array([[-np.cos(theta), np.sin(theta), 0, x],
                  [np.sin(theta), np.cos(theta), 0, y],
                  [0,                   0,        -1, z],
                  [0,                   0,         0, 1]])
    return SE3

placetimes=0
place_areay=[-0.08,0,0.08]
place_areax=[0.6,0.70,0.77]
class Baxterpicknumber():
    def __init__(self):
        global place_areax
        global place_areay
        global placetimes

        rospy.loginfo("=============Ready to move the robot================")
        
        #Initialize moveit_commander
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
        self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
        
        #Display Trajectory in RVIZ
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        self.moving_publisher = rospy.Publisher('amimoving',String, queue_size=10)
        rospy.sleep(3.0)

        self.planning_frame = self.right_arm_group.get_planning_frame()
        rospy.loginfo( "============ Reference frame: %s" % self.planning_frame)
        
        self.eef_link = self.right_arm_group.get_end_effector_link()
        print "============ End effector: %s" % self.eef_link    

        self.group_names = self.robot.get_group_names()
        print "============ Robot Groups:", self.robot.get_group_names()

        #set planning and execution parameters
        self.right_arm_group.set_goal_position_tolerance(0.01)
        self.right_arm_group.set_goal_orientation_tolerance(0.01)
        self.right_arm_group.set_planning_time(5.0)
        self.right_arm_group.allow_replanning(False)
        self.right_arm_group.set_max_velocity_scaling_factor(1)
        self.right_arm_group.set_max_acceleration_scaling_factor(1)

        self.left_arm_group.set_goal_position_tolerance(0.01)
        self.left_arm_group.set_goal_orientation_tolerance(0.01)
        self.left_arm_group.set_planning_time(5.0)
        self.left_arm_group.allow_replanning(False)
        self.left_arm_group.set_max_velocity_scaling_factor(0.5)
        self.left_arm_group.set_max_acceleration_scaling_factor(0.5)        

        #specify which gripper and limb are taking command  
        self.right_gripper = Gripper('right', CHECK_VERSION)
        self.right_gripper.reboot()
        self.right_gripper.calibrate()

        # self.left_gripper = Gripper('left', CHECK_VERSION)
        # self.left_gripper.reboot()
        # self.left_gripper.calibrate()
      
        #Standoff hight above the block
        self.standoff=0.2

        #Screw axis for Right arm of Baxter

        self.Blist=np.array([[-1, 0, 0, 0, 1.17594, 0],
                            [ 0, 1, 0, 1.10694, 0 ,-0.079],
                            [ 0, 0, 1, 0, 0.079 , 0],
                            [ 0 ,1, 0 ,0.74259, 0 ,-0.01],
                            [ 0 ,0, 1, 0, 0.01, 0],
                            [ 0, 1, 0, 0.3683, 0 ,0],
                            [ 0, 0, 1, 0 ,0 ,0]]).T

        self.M=np.array([[0, 1/sqrt(2), 1/sqrt(2), 0.064+(1.17594/sqrt(2))],
                        [0,1/sqrt(2),-1/sqrt(2), -0.278- (1.17594/sqrt(2))],
                        [-1,0,0, 0.19135],
                        [0, 0, 0,1]])

      
        #subscribe to range sensor topic
        self.__right_sensor=rospy.Subscriber('/robot/range/right_hand_range/state',Range,self.rangecallback)
        self.rangestatetemp=Range()
        #subscribe to target location topic
        self.grabcamera()
        self.go_startpose()
        self.location_data=rospy.Subscriber('square_location',String,self.compute_cartesian)
        # self.compute_cartesian()
    def grabcamera(self):
    #     self.putcamerapose= {'left_s0': -np.pi/4,
    #                         'left_s1':  -np.pi/12,
    #                         'left_e0': 0,
    #                         'left_e1': 0,
    #                         'left_w0': 0,
    #                         'left_w1': np.pi/12,
    #                         'left_w2': 0}
    #     self.left_arm_group.set_joint_value_target(self.putcamerapose)
    #     plan1=self.left_arm_group.plan()
    #     self.left_arm_group.go()
    #     rospy.loginfo("=======put camera in left gripper please=======")
    #     raw_input()
    #     self.left_gripper.close()
    #     rospy.sleep(3.0)
           
        self.holdcamerapose= {'left_s0': -0.04678641403050512,
                            'left_s1':  -1.4001409641424114,
                            'left_e0': 0.3712233506682701,
                            'left_e1': 1.3940050409908697,
                            'left_w0': -0.46326219794139495,
                            'left_w1': 2.05706823655434,
                            'left_w2': -0.943014689352558}

        self.left_arm_group.set_joint_value_target(self.holdcamerapose)
        plan2=self.left_arm_group.plan()
        self.left_arm_group.go()
        rospy.sleep(2.0)
    def rangecallback(self, data):
        self.rangestatetemp=data 
        self.rangestate=self.rangestatetemp.range
    
    def go_startpose(self):
        self.move="0"
        self.moving_publisher.publish(self.move)
        global placetimes
        # self.moving='1'
        # self.moving_publisher(self.moving)
        #Natural position of baxter
        self.initialpose = {'right_s0': 0,#0.03413107253045045,
                            'right_s1': -50*np.pi/180 ,#-0.6937428113211783,
                            'right_e0': 0,#0.1277039005914607, 
                            'right_e1': 70*np.pi/180,#0.9671748867617533,
                            'right_w0': 0,#-0.16835439147042416,
                            'right_w1': 72*np.pi/180,#1.0810729602622453,
                            'right_w2': 0}#-0.1472621556369997}
        self.right_arm_group.set_joint_value_target(self.initialpose)
        rospy.loginfo("Attempting to start pose")
        plan=self.right_arm_group.plan()
        self.right_arm_group.go()

        # rospy.sleep(1.0)
        # if placetimes>0:
        #     rospy.loginfo("==========Ready to fetch another number, Press 'Enter' to cotinue=======")
        #     raw_input()
        #     self.standbypose()
        self.standbypose()
    def standbypose(self):
        global placetimes
        if placetimes <3:
            self.placex_coord= place_areax[0]
        
            self.placey_coord= place_areay[placetimes]
        elif placetimes <6:
            self.placex_coord= place_areax[1]
            self.placey_coord= place_areay[placetimes-3]
        elif placetimes <9:
            self.placex_coord= place_areax[2]
            self.placey_coord= place_areay[placetimes-6]
        else: 
            rospy.loginfo("========There is currently no place to put block, Please press 'Enter' after cleaning the table.")
            raw_input()
            placetimes=0
            self.placex_coord= place_areax[0]
    
            self.placey_coord= place_areay[placetimes]                    
        #self.compute_cartesian()

    def compute_cartesian(self,data):
        # self.moving='0'
        # self.moving_publisher(self.moving)
        # testingdata=np.array([1, 0.55597888,-0.24354399,-0.3,0])
        # data=testingdata
        data = data.data.split('&')
        fetchnumber=float(data[0])
        # rospy.loginfo("=============Trying to fetch number   " + str(fetchnumber) + "   Press 'Enter' to confirm==============")
        confirm=raw_input("=============Trying to fetch number   " + str(fetchnumber) + "   Press 'y/n' to confirm==============")
        # confirm ='y'
        if confirm=="y":

                        
            rospy.loginfo("==============Received desired position")
            #extract x,y,z coordinate from subscriber data
            self.x_coord=float(data[1])-0.020
            # rospy.loginfo(str(self.x_coord))
            self.y_coord=float(data[2])+0.038
            # rospy.loginfo(str(self.y_coord))
            self.z_coord=-0.3
            self.theta= 0

            self.pose_target=converttoSE3(self.x_coord,self.y_coord,self.z_coord,self.theta)
            # try:
            self.move_arm_standoff()
        else:
            self.go_startpose()
        # except:
            # self.right_gripper.open()
            # self.go_startpose()
    def move_arm_standoff(self):

        pose_target=converttoSE3(self.x_coord,self.y_coord,self.z_coord+self.standoff,self.theta)
        
        theta0list=np.array([0.74,-0.67,0.2,1.2,0,1.068801113959162,0])
        joint_targettemp=IKinBody(self.Blist,self.M,pose_target,theta0list,0.01,0.001)
        

        if joint_targettemp[1]== True:
            rospy.loginfo("Solution Found! Joint target value computed successfully.")
            print(joint_targettemp[0])
            self.pickstandoff_joint_target={'right_s0': joint_targettemp[0][0],#+0.05,
                                            'right_s1': joint_targettemp[0][1],
                                            'right_e0': joint_targettemp[0][2],
                                            'right_e1': joint_targettemp[0][3],
                                            'right_w0': joint_targettemp[0][4],
                                            'right_w1': joint_targettemp[0][5],
                                            'right_w2': joint_targettemp[0][6]}
            # try:

            self.right_arm_group.set_joint_value_target(self.pickstandoff_joint_target)
            rospy.loginfo("Attempting to go to pick stand off position")
            plan=self.right_arm_group.plan()
            self.right_arm_group.go()
            rospy.sleep(1.0)
            self.move_arm_pick()

            # except:
            #     self.right_gripper.open()
            #     self.go_startpose()
            
            
        else:
            rospy.logerr("Could not find valid joint target value, returnning to Initial position")
            self.right_gripper.open()
            self.go_startpose()
        
        
        
    def move_arm_pick(self):
        pose_target=converttoSE3(self.x_coord,self.y_coord,self.z_coord,self.theta)
        theta0list=np.array([self.pickstandoff_joint_target['right_s0'],
                            self.pickstandoff_joint_target['right_s1'],
                            self.pickstandoff_joint_target['right_e0'],
                            self.pickstandoff_joint_target['right_e1'],
                            self.pickstandoff_joint_target['right_w0'],
                            self.pickstandoff_joint_target['right_w1'],
                            self.pickstandoff_joint_target['right_w2']])
        
        joint_targettemp=IKinBody(self.Blist,self.M,pose_target,theta0list,0.01,0.001)

        if joint_targettemp[1]== True:
            rospy.loginfo("Solution Found! Joint target value computed successfully.")
            print(joint_targettemp[0])
            self.pick_joint_target={'right_s0': joint_targettemp[0][0],
                                    'right_s1': joint_targettemp[0][1],
                                    'right_e0': joint_targettemp[0][2],
                                    'right_e1': joint_targettemp[0][3],
                                    'right_w0': joint_targettemp[0][4],
                                    'right_w1': joint_targettemp[0][5],
                                    'right_w2': joint_targettemp[0][6]}

            self.right_arm_group.set_joint_value_target(self.pick_joint_target)
            rospy.loginfo("Attempting to go to pick position")
            plan=self.right_arm_group.plan()
            self.right_arm_group.go()
            rospy.sleep(1.0)
            print(self.rangestate)
            if self.rangestate < 0.2:
                self.right_gripper.close()
            else:
                self.right_gripper.open()
            self.move_arm_backstandoff()
        else:
            rospy.logerr("Could not find valid joint target value, returnning to Initial position")
            self.right_gripper.open()
            self.go_startpose()
        
        # self.move_arm_backstandoff()
    def move_arm_backstandoff(self):
        theta0list=np.array([self.pick_joint_target['right_s0'],
                            self.pick_joint_target['right_s1'],
                            self.pick_joint_target['right_e0'],
                            self.pick_joint_target['right_e1'],
                            self.pick_joint_target['right_w0'],
                            self.pick_joint_target['right_w1'],
                            self.pick_joint_target['right_w2']])
        pose_target=converttoSE3(self.x_coord,self.y_coord,self.z_coord+self.standoff,self.theta)
        
        joint_targettemp=IKinBody(self.Blist,self.M,pose_target,theta0list,0.01,0.001)
        

        if joint_targettemp[1]== True:
            rospy.loginfo("Solution Found! Joint target value computed successfully.")
            print(joint_targettemp[0])
            self.pickbackstandoff_joint_target={'right_s0': joint_targettemp[0][0],
                                            'right_s1': joint_targettemp[0][1],
                                            'right_e0': joint_targettemp[0][2],
                                            'right_e1': joint_targettemp[0][3],
                                            'right_w0': joint_targettemp[0][4],
                                            'right_w1': joint_targettemp[0][5],
                                            'right_w2': joint_targettemp[0][6]}

            self.right_arm_group.set_joint_value_target(self.pickbackstandoff_joint_target)
            rospy.loginfo("Attempting to go back to pick standoff position")
            plan=self.right_arm_group.plan()
            self.right_arm_group.go()
            rospy.sleep(1.0)
            if self.rangestate< 0.3:
                self.right_gripper.close()
                self.move_arm_standoff2()

            else:
                rospy.loginfo("=========Sorry, I drop the block, returning to start pose=====")
                self.right_gripper.open()
                self.go_startpose()
            
        else:
            rospy.logerr("Could not find valid joint target value, returnning to Initial position")
            self.right_gripper.open()
            self.go_startpose()
        
        

    def move_arm_standoff2(self):
        theta0list=np.array([self.pickbackstandoff_joint_target['right_s0'],
                            self.pickbackstandoff_joint_target['right_s1'],
                            self.pickbackstandoff_joint_target['right_e0'],
                            self.pickbackstandoff_joint_target['right_e1'],
                            self.pickbackstandoff_joint_target['right_w0'],
                            self.pickbackstandoff_joint_target['right_w1'],
                            self.pickbackstandoff_joint_target['right_w2']])
        
        # theta0list=np.array([0.8,-0.64,0.25,1.0400389741863105,0.2,1.068801113959162,0])
        pose_target=converttoSE3(self.placex_coord,self.placey_coord,self.z_coord+self.standoff,self.theta)
        
        joint_targettemp=IKinBody(self.Blist,self.M,pose_target,theta0list,0.01,0.001)
        

        if joint_targettemp[1]== True:
            rospy.loginfo("Solution Found! Joint target value computed successfully.")
            print(joint_targettemp[0])
            self.placestandoff_joint_target={'right_s0': joint_targettemp[0][0],
                                            'right_s1': joint_targettemp[0][1],
                                            'right_e0': joint_targettemp[0][2],
                                            'right_e1': joint_targettemp[0][3],
                                            'right_w0': joint_targettemp[0][4],
                                            'right_w1': joint_targettemp[0][5],
                                            'right_w2': joint_targettemp[0][6]}

            self.right_arm_group.set_joint_value_target(self.placestandoff_joint_target)
            rospy.loginfo("Attempting to go to place stand off position")
            plan=self.right_arm_group.plan()
            self.right_arm_group.go()
            rospy.sleep(1.0)
            if self.rangestate< 0.3:
                self.right_gripper.close()
                self.move_arm_place()
            else:
                rospy.loginfo("Sorry, I drop the block, return to start pose")
                self.right_gripper.open()
                self.go_startpose()
            
        else:
            rospy.logerr("Could not find valid joint target value, returnning to Initial position")
            self.right_gripper.open()
            self.go_startpose() 
        # rospy.sleep(1.0)       
        
    def move_arm_place(self):
        global placetimes
        pose_target=converttoSE3(self.placex_coord,self.placey_coord,self.z_coord,0)
        theta0list=np.array([self.placestandoff_joint_target['right_s0'],
                            self.placestandoff_joint_target['right_s1'],
                            self.placestandoff_joint_target['right_e0'],
                            self.placestandoff_joint_target['right_e1'],
                            self.placestandoff_joint_target['right_w0'],
                            self.placestandoff_joint_target['right_w1'],
                            self.placestandoff_joint_target['right_w2']])
        
        joint_targettemp=IKinBody(self.Blist,self.M,pose_target,theta0list,0.01,0.001)

        if joint_targettemp[1]== True:
            rospy.loginfo("Solution Found! Joint target value computed successfully.")
            print(joint_targettemp[0])
            self.place_joint_target={'right_s0': joint_targettemp[0][0],
                                    'right_s1': joint_targettemp[0][1],
                                    'right_e0': joint_targettemp[0][2],
                                    'right_e1': joint_targettemp[0][3],
                                    'right_w0': joint_targettemp[0][4],
                                    'right_w1': joint_targettemp[0][5],
                                    'right_w2': joint_targettemp[0][6]}

            self.right_arm_group.set_joint_value_target(self.place_joint_target)
            rospy.loginfo("Attempting to go to pick position")
            plan=self.right_arm_group.plan()
            self.right_arm_group.go()
            placetimes = placetimes+1 
            self.right_gripper.open()
            # print(placetimes)
            self.move_arm_backstandoff2()
        else:
            rospy.logerr("Could not find valid joint target value")   
            self.go_startpose()
                   
        
    def move_arm_backstandoff2(self):
        theta0list=np.array([self.place_joint_target['right_s0'],
                            self.place_joint_target['right_s1'],
                            self.place_joint_target['right_e0'],
                            self.place_joint_target['right_e1'],
                            self.place_joint_target['right_w0'],
                            self.place_joint_target['right_w1'],
                            self.place_joint_target['right_w2']])
        pose_target=converttoSE3(self.placex_coord,self.placey_coord,self.z_coord+self.standoff,0)
        
        # theta0list=np.array([0.74,-0.585980660972228,0,1.0400389741863105,0,1.068801113959162,0])
        joint_targettemp=IKinBody(self.Blist,self.M,pose_target,theta0list,0.01,0.001)
        

        if joint_targettemp[1]== True:
            rospy.loginfo("Solution Found! Joint target value computed successfully.")
            print(joint_targettemp[0])
            self.placebackstandoff_joint_target={'right_s0': joint_targettemp[0][0],
                                            'right_s1': joint_targettemp[0][1],
                                            'right_e0': joint_targettemp[0][2],
                                            'right_e1': joint_targettemp[0][3],
                                            'right_w0': joint_targettemp[0][4],
                                            'right_w1': joint_targettemp[0][5],
                                            'right_w2': joint_targettemp[0][6]}

            self.right_arm_group.set_joint_value_target(self.placebackstandoff_joint_target)
            rospy.loginfo("Attempting to go to place stand off position")
            plan=self.right_arm_group.plan()
            self.right_arm_group.go()
            
            self.right_gripper.open()
            self.go_startpose()
            
        else:
            rospy.logerr("Could not find valid joint target value")  
            self.go_startpose()

def main():
    rospy.init_node("moveit_baxter_pickandplace",anonymous=True)
    
    try:
       
        movearm = Baxterpicknumber()
    except rospy.ROSInterruptException: pass

    rospy.spin()
    
if __name__ == '__main__':
	main()