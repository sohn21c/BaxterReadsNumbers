#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
import numpy as np
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
import baxter_interface
from baxter_interface import Gripper ,CHECK_VERSION
from sensor_msgs.msg import Range
import tf.transformations as tr


class Baxterpicknumber():
    def __init__(self):
        rospy.loginfo("++++++++++++Ready to move the robot+++++++++++")
        
        #Initialize moveit_commander
        self.robot=moveit_commander.RobotCommander()
        self.scene=moveit_commander.PlanningSceneInterface()
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
        
        #Display Trajectory in RVIZ
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        self.moving_publisher = rospy.Publisher('amimoving',
                                                  String, queue_size=10)
        rospy.sleep(3.0)
        # print self.robot.get_current_state()
        # print ""

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
        self.right_arm_group.set_max_velocity_scaling_factor(0.6)
        self.right_arm_group.set_max_acceleration_scaling_factor(0.6)

        #specify which gripper and limb are taking command  
        self.right_gripper = Gripper('right', CHECK_VERSION)
        self.right_gripper.reboot()
        self.right_gripper.calibrate()

        # self.limb = baxter_interface.Limb('right')
        # self.angles = self.limb.joint_angles()
        # self.wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
        # rospy.sleep(3)
        # self.limb.move_to_joint_positions(self.wave_1)


        self.pose_target=Pose()
        self.standoff=0.1
        self.__right_sensor=rospy.Subscriber('/robot/range/right_hand_range/state',Range,self.rangecallback)
        self.rangestatetemp=Range()

        self.box_name='table'
        self.moving='0'
        self.moving_publisher.publish(self.moving)
            
    def add_box(self,timeout=4):

        self.box_pose=PoseStamped()
        self.box_pose.header.frame_id="base"
        self.box_pose.pose.position.x=1.0
        self.box_pose.pose.position.z=-0.27
        self.box_pose.pose.orientation.w=1.0
        self.scene.add_box(self.box_name,self.box_pose,size=(1.5,2,0.1))

        # return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False        

    

    def rangecallback(self, data):
        self.rangestatetemp=data 
        self.rangestate=self.rangestatetemp.range
        

    def compute_cartesian(self):#,data):
        testingdata=np.array([0.6,-0.320147457674,-0.209,np.pi])
        data=testingdata
        
        rospy.loginfo("++++++++++Received desired position+++++++++")
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
        self.cartesian_reader=True
    
    # def move_arm_standoff_cartesianpath(self):
    #     self.moving=True
    #     waypoints=[]
    #     rospy.sleep(1.0)
    #     wpose=self.pose_target
    #     wpose.posistion.z= self.pose_target
    #     wpose.position=self.quat_position
    #     waypoints.append(copy.deepcopy(wpose))
    #     print('moving to standoff')

    #     (plan,fraction)=self.right_arm_group.compute_cartesian_path(waypoints,0.01,0.0)

    #     self.right_arm_group.execute(plan,wait=True)
    def move_arm_standoff(self):
        self.moving='1'
        self.moving_publisher.publish(self.moving)
        rospy.loginfo("+++++++++++++Going to standoff posistion+++++++++")
        #def standoff height (above the block)
        
        #standoff position has same gripper orentation as block orientation
        self.pose_target.orientation=self.quat_orientation
        #add standoff height to the height of the block and get new pose_target.position
        z_standoff=self.z_coord+self.standoff
        pout=np.array([self.x_coord,self.y_coord,z_standoff])
        self.pose_target.position=Point(*pout)

        #Calling Moveit to use IK to compute the plan and execute it
        print(self.pose_target)
        self.right_arm_group.set_pose_target(self.pose_target)
        rospy.sleep(3.0)
        plan_standoff=self.right_arm_group.go(wait=True)

        self.right_arm_group.stop()
        self.right_arm_group.clear_pose_targets()
        rospy.sleep(1.0)


    # def move_arm_pick_cartesianpath(self,scale=1):
  
        
    #     waypoints=[]
    #     rospy.sleep(1.0)
    #     wpose=self.pose_target
    #     wpose.position=self.quat_position
    #     waypoints.append(copy.deepcopy(wpose))
    #     print('moving to pick up')

    #     (plan,fraction)=self.right_arm_group.compute_cartesian_path(waypoints,0.01,0.0)

    #     self.right_arm_group.execute(plan,wait=True)
     
    def remove_box(self):
        self.scene.remove_world_object(self.box_name)

    def move_arm_pick(self):

        
    
        rospy.loginfo("+++++++++++++Going to pickup posistion+++++++++")
        

        #standoff position has same gripper orentation and height as block orientation
        self.pose_target.orientation=self.quat_orientation

        
        z_standoff=self.z_coord
        pout=np.array([self.x_coord,self.y_coord,z_standoff])
        self.pose_target.position=Point(*pout)

        #Calling Moveit to use IK to compute the plan and execute it
        print(self.pose_target)
        self.right_arm_group.set_pose_target(self.pose_target)
        rospy.sleep(2.0)
        plan_standoff=self.right_arm_group.go(wait=True)

        print(self.rangestate)
        if self.rangestate < 0.4:
            self.right_gripper.close()
        else:
            self.right_gripper.open()
        self.right_arm_group.stop()
        self.right_arm_group.clear_pose_targets()
        rospy.sleep(1.0)  

    # def move_arm_back_standoff_cartesianpath(self,scale=1):

    #     waypoints=[]
    #     rospy.sleep(1.0)

    #     wpose=self.pose_target
        
    #     waypoints.append(copy.deepcopy(wpose))

    #     (plan,fraction)=self.right_arm_group.compute_cartesian_path(waypoints,0.005,0.01)

    #     self.right_arm_group.execute(plan,wait=True)
        
    def move_arm_back_standoff(self):
        
        rospy.loginfo("+++++++++++++Going back to stand off posistion+++++++++")
        self.right_gripper.close()
        #def standoff height (above the block)
        
        #standoff position has same gripper orentation as block orientation
        self.pose_target.orientation=self.quat_orientation
        #add standoff height to the height of the block and get new pose_target.position
        z_standoff=self.z_coord+self.standoff
        pout=np.array([self.x_coord,self.y_coord,z_standoff])
        self.pose_target.position=Point(*pout)

        #Calling Moveit to use IK to compute the plan and execute it
        print(self.pose_target)
        self.right_arm_group.set_pose_target(self.pose_target)
        rospy.sleep(2.0)
        plan_backtostandoff=self.right_arm_group.go(wait=True)

        self.right_arm_group.stop()
        self.right_arm_group.clear_pose_targets()
        rospy.sleep(1.0)
    
    # def move_arm_standoff2_cartesianpath(self,scale=1):

    #     self.pose_target.position=self.quat_position


    #     waypoints=[]
    #     rospy.sleep(1.0)

    #     wpose=self.pose_target
    #     wpose.position.z = scale * self.standoff
    #     waypoints.append(copy.deepcopy(wpose))

    #     (plan,fraction)=self.right_arm_group.compute_cartesian_path(waypoints,0.01,0.0)

    #     self.right_arm_group.execute(plan,wait=True)


    def move_arm_standoff2(self):
        
        rospy.loginfo("+++++++++++++Going to standoff2 posistion+++++++++")
        #def standoff height (above the block)
        self.right_gripper.close()
        
        #standoff position has same gripper orentation as block orientation
        self.pose_target.orientation=self.quat_orientation
        #add standoff height to the height of the block and get new pose_target.position
        x_standoff2=self.x_coord
        z_standoff2=self.z_coord+self.standoff
        y_standoff2=self.y_coord+0.2
        pout=np.array([x_standoff2,y_standoff2,z_standoff2])
        self.pose_target.position=Point(*pout)

        #Calling Moveit to use IK to compute the plan and execute it
        print(self.pose_target)
        self.right_arm_group.set_pose_target(self.pose_target)
        rospy.sleep(3.0)
        plan_standoff2=self.right_arm_group.go(wait=True)

        self.right_arm_group.stop()
        self.right_arm_group.clear_pose_targets()
        rospy.sleep(1.0)

    # def move_arm_place_cartesianpath(self,scale=1):
  
        
    #     waypoints=[]
    #     rospy.sleep(1.0)

    #     wpose=self.pose_target
    #     wpose.position.z = self.quat_position.z
    #     waypoints.append(copy.deepcopy(wpose))

    #     (plan,fraction)=self.right_arm_group.compute_cartesian_path(waypoints,0.005,0.01)

    #     self.right_arm_group.execute(plan,wait=True)

    def move_arm_place(self):
        
        global placedtimes
        rospy.loginfo("+++++++++++++Going to place posistion+++++++++")
        
        self.right_gripper.close()
        #standoff position has same gripper orentation and height as block orientation
        self.pose_target.orientation=self.quat_orientation

        # z_standoff=block_height/2  in case need gripper half height higer than predicted block height.
        
        self.pose_target.position=self.quat_position
        self.placetarget_y= np.linspace(0.2,0.29,10)
        x_place=self.x_coord
        z_place=self.z_coord
        y_place=self.y_coord+self.placetarget_y[placedtimes]

        
            
        pout=np.array([x_place,y_place,z_place])
        self.pose_target.position=Point(*pout)

        
        #Calling Moveit to use IK to compute the plan and execute it
        print(self.pose_target)
        self.right_arm_group.set_pose_target(self.pose_target)
        rospy.sleep(2.0)
        plan_place=self.right_arm_group.go(wait=True)


        self.right_gripper.open()
        self.right_arm_group.stop()
        self.right_arm_group.clear_pose_targets()
        rospy.sleep(1.0)   

    def move_arm_backstandoff2(self):
        global placedtimes
        rospy.loginfo("+++++++++++++Going back to standoff2 posistion+++++++++")
        #def standoff height (above the block)
        self.right_gripper.open()
        
        #standoff position has same gripper orentation as block orientation
        self.pose_target.orientation=self.quat_orientation
        #add standoff height to the height of the block and get new pose_target.position
        x_standoff2=self.x_coord
        z_standoff2=self.z_coord+self.standoff
        y_standoff2=self.y_coord+self.placetarget_y[placedtimes]
        pout=np.array([x_standoff2,y_standoff2,z_standoff2])
        self.pose_target.position=Point(*pout)

        #Calling Moveit to use IK to compute the plan and execute it
        print(self.pose_target)
        self.right_arm_group.set_pose_target(self.pose_target)
        rospy.sleep(2.0)
        plan_backstandoff2=self.right_arm_group.go(wait=True)

        self.right_arm_group.stop()
        self.right_arm_group.clear_pose_targets()
        rospy.sleep(1.0)
        placedtimes += 1
        self.moving='0'
        self.moving_publisher.publish(self.moving)

#     def move_arm_back_standoff2_cartesianpath(self,scale=1):

        


#         waypoints=[]
#         rospy.sleep(1.0)

#         wpose=self.pose_target
#         # wpose.position.z += scale * self.standoff
#         waypoints.append(copy.deepcopy(wpose))

#         (plan,fraction)=self.right_arm_group.compute_cartesian_path(waypoints,0.005,0.01)

#         self.right_arm_group.execute(plan,wait=True)        

placedtimes=0

def main():
    rospy.init_node("moveit_baxter_pickandplace",anonymous=True)
    
    
    #listen to the topic which is publishing desired cartian coordinate
    
    # rospy.Subscriber("square-location",String,self.compute_cartesian)
    
    try:
        
        
        # print("===========Testing start========")
        # raw_input()        
        movearm = Baxterpicknumber()
        # print("=========recieve desired position====")
        # raw_input()
 
        movearm.add_box()
        movearm.compute_cartesian()
        # # print("========go to standoff position=====")
        # # raw_input()
        
        movearm.move_arm_standoff()
        # movearm.remove_box()
        # # print("=====go pickup======")
        # # raw_input()
        movearm.move_arm_pick()
        
        # movearm.move_arm_pick()
        # # print("=====go back to standoff=======")
        # # raw_input()
        movearm.move_arm_back_standoff()
        # movearm.add_box()
        
        # # print("======go to standoff 2=====")
        # # raw_input()
        movearm.move_arm_standoff2()
        # movearm.remove_box()
        # # print("=========go place the block====")
        # # raw_input()
        movearm.move_arm_place()
        
        # # print('========go back to standoff2 ====')
        # # raw_input()
        movearm.move_arm_backstandoff2()
        
        # # print('====back to initial position=====')
        
    except rospy.ROSInterruptException: pass

    rospy.spin()
    
if __name__ == '__main__':
	main()