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


class Baxterpicknumber():
    def __init__(self):
        rospy.loginfo("=============Ready to move the robot================")
        
        #Initialize moveit_commander
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
        
        #Display Trajectory in RVIZ
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

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
        self.right_arm_group.set_max_velocity_scaling_factor(0.5)
        self.right_arm_group.set_max_acceleration_scaling_factor(0.5)

        #specify which gripper and limb are taking command  
        self.right_gripper = Gripper('right', CHECK_VERSION)
        self.right_gripper.reboot()
        self.right_gripper.calibrate()

        self.pose_target=Pose()
        self.startpose=Pose()
        self.standoff=0.2

        #Where to place the block
        self.place_target=Pose()
        self.placex_coord= 0.65
        self.placey_coord=0

        self.__right_sensor=rospy.Subscriber('/robot/range/right_hand_range/state',Range,self.rangecallback)
        # self.location_data=rospy.Subscriber('square_location',String,self.compute_cartesian)
        self.compute_cartesian()
        self.rangestatetemp=Range()
        

    def rangecallback(self, data):
        self.rangestatetemp=data 
        self.rangestate=self.rangestatetemp.range
    
    def go_startpose(self):
        self.initialpose = {'right_s0': 0.03413107253045045,
                            'right_s1': -0.6937428113211783,
                            'right_e0': 0.1277039005914607, 
                            'right_e1': 0.9671748867617533,
                            'right_w0': -0.16835439147042416,
                            'right_w1': 1.0810729602622453,
                            'right_w2': -0.1472621556369997}
        self.right_arm_group.set_joint_value_target(self.initialpose)
        rospy.loginfo("Attempting to start pose")
        plan=self.right_arm_group.plan()
        self.right_arm_group.go()

        
        self.startpose.position.x=0.693292944176
        self.startpose.position.y=-0.81004861946
        self.startpose.position.z=0.0914586599661
        self.startpose.orientation.x=0.235668914045
        self.startpose.orientation.y=0.964651313563
        self.startpose.orientation.z=-0.0681154565907
        self.startpose.orientation.w=0.0962719625176

    def compute_cartesian(self):#,data):
        testingdata=np.array([0.738492350508,-0.320147457674,-0.209,np.pi])
        data=testingdata
        # data=data.data.split('&')
        
        rospy.loginfo("=================Received desired position==============")
        #extract x,y,z coordinate from subscriber data
        self.x_coord=data[0]
        self.y_coord=data[1]
        self.z_coord=data[2]
        theta=  data[3]

        #put coordinate data back to arrays
        pout=np.array([self.x_coord,self.y_coord,self.z_coord])
        block_orientation=tr.quaternion_from_euler(theta,0,0,'sxyz')
        #convert orentation and position to Quaternion
        quat=Quaternion(*block_orientation)

        self.quat_position=Point(*pout)
        self.quat_orientation= copy.deepcopy(quat)
        self.pose_target.position=Point(*pout)
        self.pose_target.orientation=self.quat_orientation
        #Quaternian location for place area
        placepout=np.array([self.placex_coord,self.placey_coord,self.z_coord])
        self.place_target.position=Point(*placepout)
        self.place_target.orientation=copy.deepcopy(quat)


    def move_arm_standoff(self):
        # self.moving='1'
        # self.moving_publisher.publish(self.moving)
        rospy.loginfo("==================Going to standoff posistion==============")

        z_standoff=self.z_coord+self.standoff
               

        waypoints=[]
        ite=30
        xite=np.linspace(self.startpose.position.x,self.x_coord,ite)
        yite=np.linspace(self.startpose.position.y,self.y_coord,ite)
        zite=np.linspace(self.startpose.position.z,z_standoff,ite)
        # xoite=np.linspace(self.startpose.orientation.x,self.quat_orientation.x,ite)
        # yoite=np.linspace(self.startpose.orientation.y,self.quat_orientation.y,ite)
        # zoite=np.linspace(self.startpose.orientation.z,self.quat_orientation.z,ite)
        # woite=np.linspace(self.startpose.orientation.w,self.quat_orientation.w,ite)

        for i in range(ite):
            p=copy.deepcopy(self.startpose)
            p.position.x=xite[i]
            p.position.y=yite[i]
            p.position.z=zite[i]
            # p.orientation.x=xoite[i]
            # p.orientation.y=yoite[i]
            # p.orientation.z=zoite[i]
            # p.orientation.w=woite[i]
            waypoints.append(p)
        
        # print(waypoints)
        rospy.sleep(1.0)

        self.right_arm_group.set_start_state_to_current_state()
        fraction = 0.0 
        max_attempts = 200 
        attempts = 0 

        # Plan the Cartesian path connecting the waypoints 
        while fraction < 1.0 and attempts < max_attempts: 
            (plan, fraction) = self.right_arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            # Increment the number of attempts 
            attempts += 1 
            # Print out a progress message 
            if attempts % 10 == 0: 
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 
        # If we have a complete plan, execute the trajectory 
        if fraction == 1.0: 
            rospy.loginfo("Path computed successfully. Moving the arm.") 
            self.right_arm_group.execute(plan)
            
        else:
            rospy.logerr("Could not find valid cartesian path")
            self.go_startpose()

        # self.move_arm_pick()
        
    def move_arm_pick(self):

        
        rospy.loginfo("====================moving to pick===================") 
        waypoints=[]
        
        z_standoff=self.z_coord+self.standoff
        ite=30
        zite=np.linspace(z_standoff,self.z_coord,ite)
        waypoints=[]
        # print(zite)
        for i in range(ite):
            p=copy.deepcopy(self.pose_target)
            p.position.z=zite[i]
    
            waypoints.append(p)
        # print(waypoints)
        rospy.sleep(1.0)
        self.right_arm_group.set_start_state_to_current_state()
        fraction = 0.0 
        max_attempts = 200 
        attempts = 0 

        # Plan the Cartesian path connecting the waypoints 
        while fraction < 1.0 and attempts < max_attempts: 
            (plan, fraction) = self.right_arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            # Increment the number of attempts 
            attempts += 1 
            # Print out a progress message 
            if attempts % 10 == 0: 
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 
        # If we have a complete plan, execute the trajectory 
        if fraction == 1.0: 
            rospy.loginfo("Path computed successfully. Moving the arm.") 
            self.right_arm_group.execute(plan)
        else:
            rospy.logerr("Could not find valid cartesian path")
        
        # print(self.rangestate)
        if self.rangestate < 0.4:
            self.right_gripper.close()
        else:
            self.right_gripper.open()
        # self.right_arm_group.stop()
        # self.right_arm_group.clear_pose_targets()
        # rospy.sleep(1.0)      

    def move_arm_backstandoff(self):
        self.right_gripper.close()
        rospy.loginfo('====================moving back to standoff=================') 
        
        waypoints=[]
        
        z_standoff=self.z_coord+self.standoff
        ite=30
        zite=np.linspace(self.z_coord,z_standoff,30)
        waypoints=[]
        # print(zite)
        for i in range(ite):
            p=copy.deepcopy(self.pose_target)
            p.position.z=zite[i]
    
            waypoints.append(p)
        # print(waypoints)
        rospy.sleep(1.0)
        self.right_arm_group.set_start_state_to_current_state()
        fraction = 0.0 
        max_attempts = 200 
        attempts = 0 

        # Plan the Cartesian path connecting the waypoints 
        while fraction < 1.0 and attempts < max_attempts: 
            (plan, fraction) = self.right_arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            # Increment the number of attempts 
            attempts += 1 
            # Print out a progress message 
            if attempts % 10 == 0: 
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 
        # If we have a complete plan, execute the trajectory 
        if fraction == 1.0: 
            rospy.loginfo("Path computed successfully. Moving the arm.") 
            self.right_arm_group.execute(plan)
        else:
            rospy.logerr("Could not find valid cartesian path")            
    
    def move_arm_standoff2(self):
        rospy.loginfo('==================moving to place standoff position==================') 
        waypoints=[]
        
        z_standoff=self.z_coord+self.standoff
        ite=30
        xite=np.linspace(self.x_coord,self.placex_coord,ite)
        yite=np.linspace(self.y_coord,self.placey_coord,ite)
        waypoints=[]
        # print(zite)
        for i in range(ite):
            p=copy.deepcopy(self.place_target)
            p.position.z=z_standoff
            p.position.x=xite[i]
            p.position.y=yite[i]
            waypoints.append(p)

        # print(waypoints)
        rospy.sleep(1.0)
        self.right_arm_group.set_start_state_to_current_state()
        fraction = 0.0 
        max_attempts = 200 
        attempts = 0 

        # Plan the Cartesian path connecting the waypoints 
        while fraction < 1.0 and attempts < max_attempts: 
            (plan, fraction) = self.right_arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            # Increment the number of attempts 
            attempts += 1 
            # Print out a progress message 
            if attempts % 10 == 0: 
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 
        # If we have a complete plan, execute the trajectory 
        if fraction == 1.0: 
            rospy.loginfo("Path computed successfully. Moving the arm.") 
            self.right_arm_group.execute(plan)
        else:
            rospy.logerr("Could not find valid cartesian path")
    
    def move_arm_place(self):
        rospy.loginfo('===================moving to place the block===================') 
        waypoints=[]
        
        z_standoff=self.z_coord+self.standoff
        ite=30
        zite=np.linspace(z_standoff,self.z_coord,ite)
        waypoints=[]
        # print(zite)
        for i in range(ite):
            p=copy.deepcopy(self.place_target)
            p.position.z=zite[i]
    
            waypoints.append(p)
        # print(waypoints)
        rospy.sleep(1.0)
        self.right_arm_group.set_start_state_to_current_state()
        fraction = 0.0 
        max_attempts = 200 
        attempts = 0 

        # Plan the Cartesian path connecting the waypoints 
        while fraction < 1.0 and attempts < max_attempts: 
            (plan, fraction) = self.right_arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            # Increment the number of attempts 
            attempts += 1 
            # Print out a progress message 
            if attempts % 10 == 0: 
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 
        # If we have a complete plan, execute the trajectory 
        if fraction == 1.0: 
            rospy.loginfo("Path computed successfully. Moving the arm.") 
            self.right_arm_group.execute(plan)
        else:
            rospy.logerr("Could not find valid cartesian path")
        self.right_gripper.open()   

    def move_arm_backstandoff2(self):
        rospy.loginfo('=============moving to place standoff position=================') 
        waypoints=[]
        
        z_standoff=self.z_coord+self.standoff
        ite=30
        zite=np.linspace(self.z_coord,z_standoff,ite)
        
        waypoints=[]
        # print(zite)
        for i in range(ite):
            p=copy.deepcopy(self.place_target)
            p.position.z=zite[i]
            waypoints.append(p)

        # print(waypoints)
        rospy.sleep(1.0)
        self.right_arm_group.set_start_state_to_current_state()
        fraction = 0.0 
        max_attempts = 200 
        attempts = 0 

        # Plan the Cartesian path connecting the waypoints 
        while fraction < 1.0 and attempts < max_attempts: 
            (plan, fraction) = self.right_arm_group.compute_cartesian_path(waypoints, 0.01, 0.0)
            # Increment the number of attempts 
            attempts += 1 
            # Print out a progress message 
            if attempts % 10 == 0: 
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 
        # If we have a complete plan, execute the trajectory 
        if fraction == 1.0: 
            rospy.loginfo("Path computed successfully. Moving the arm.") 
            self.right_arm_group.execute(plan)
        else:
            rospy.logerr("Could not find valid cartesian path")
        
        self.go_startpose()
def main():
    rospy.init_node("moveit_baxter_pickandplace",anonymous=True)
    
    try:
       
        movearm = Baxterpicknumber()

        movearm.go_startpose()
        movearm.move_arm_standoff()

        movearm.move_arm_pick()

        movearm.move_arm_backstandoff()
        movearm.move_arm_standoff2()
        movearm.move_arm_place()
        movearm.move_arm_backstandoff2()
    except rospy.ROSInterruptException: pass

    rospy.spin()
    
if __name__ == '__main__':
	main()