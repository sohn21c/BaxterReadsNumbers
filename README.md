## BaxterReadsNumbers
#### ME 495: Embedded Systems in Robotics
#### _Jordan Haskel, Lukai Jin, Jingyan Ling, James Sohn, Yichi Zhang_


## Introduction

This repo contains ROS packages needed to allow Baxter, manufactured by Rethink Robotics, to perform robust block pick and place using single digit handwritten numbers on the object.  

![baxterreadsnumbers](https://github.com/sohn21c/BaxterReadsNumbers/blob/master/images/IMG_3680.PNG?raw=true)

## Hardware
- Baxter 
- 1080p usb camera
- Yellow 3x3 Post-it (10 ea.)
- Pink 3x3 Post-it (1 ea.)
- Black marker

## Software requirement
- Linux 18.04
- ROS Melodic  
- OpenCV2

## Highlevel description and included packages/files
### Required package list
- [MoveIt](https://moveit.ros.org/)
- [trac_ik](http://wiki.ros.org/trac_ik)
- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)
- [camera_info_manager_node](https://github.com/NU-MSR/camera_info_manager_node)
- [camera_info_manager_py](http://wiki.ros.org/camera_info_manager_py)

### High level description
The functionality of the package consists of two individual packages that are not listed in the required package list.  
`baxterpicknumber` and `number_rec`. Below is the simpe explanation of what each package does.
- `numberrecog`: Uses machine learning to predict which blocks say what number.
- `baxterpicknumber`: Subscribes to the topics posted by `vision.py` and calls the blockLocator service displaying relative position of the object in the external usb camera frame, convert it to position of the object in real world and perform pick and place

### Package breakdown


##### Services
`blockLocator.srv`: this service uses a string to request a number and retruns a string with the x pixel co-ordinate, y co-ordinate, the width and the length of teh block in that order all seperated by '&'s

#### `baxterpicknumber`  
##### Launch files
`roslaunch baxter_fun numberrecog.launch`: Starts usb_cam with the required arguments, It also starts the vision.py node.

`move_joint_target.launch`: initiates MoveIt!, Baxterinterface and Rviz

##### Nodes
`vision.py`: This node subscribes to the topic posted by usb_cam from the overhead usb camera and uses opencv to extract the yellow squares and preprocess them so that they can be used for number recognition that was trained on MNIST numbers (28 pixels by 28 pixels, white number on black background) It also finds the pink square in pixel co-ordinates which is used for localisation to the ar tag. This node provides the pixel co-ordinates of a given number via the blockLocator service, It also publishes the poxel co-ordinates of the pink block to `/pink_block_loc` and the size of the pink block to pink to `/pink_block` 

`move_joint_target.py`: uses [Modern_robotics library ](https://github.com/NxRLab/ModernRobotics) to solve inverse kinematics to maneuver Baxter's right gripper to designated location, grab the object and place it to designated drop off zone.  

`pixelconvert.py`: converts the object location in the usb camera pixel frame into Baxter's world frame. Conversion is done in multiple steps of matrix transformation and tracking of AR tag. Both usb camera and built-in camera in Baxter's left hand track the AR tag providing the relationship between usb camera and Baxter coordinate. Then usb camera locates pik Post-It in the frame to build conversion factor between pixel in the usb camera and the real-world coordinate. The node accepts single-digit number as user input and perform locating, picking up and moving.  

`pixelconvert2.py`: has additional function in the node that decomposes double-digit user input to multiplication of multiple single digit numbers and perform locating, pickng up and moving.  

`move_cartesianpath.py`(optional): is another approach that one can choose to work with instead of `move_joint_target.py` using moveit's cartesianpath computation method.  

`move_motionplann.py`(optional): is another approach to motion plan with moveit's built-in motion planner.  


## Demo video and package run instruction
### Demo video
One can find demo video [here](https://drive.google.com/file/d/18wlpZJT8PQiyQPO6wGeEcdG6VCEspOpR/view)  

### Package run instruction
Listed below is the series of commands to initiate the packages and run the required nodes and launch files.
- Connection to Baxter: One can find detailed insturction of workstation setup [here](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)  
  
- Launch usb_cam and number recognition (`vision.py`) node   
    `roslaunch baxter_fun numberrecog.launch`  
    
- Launch coordinate conversion  node  
    `rosrun baxterpicknumber pixelconvert.py`  
    
- Launch move and pick node  
    `roslaunch baxterpicknumber move_joint_target.launch`  
 
- Run instruction after launch  
  - Headup camera will show predictions of numbers in the workspace  
  - Command line asks for remapping. If the number prediction looks correct, press `n` if not, press `y` for refresh.
  - Input single digit or double digit number depending on `pixelconvert` node one chooses to run.  
  - Confirm the number again by pressing enter
  
  
## Issues 
Listed below are issues may affect demo and proceed accuracy.
- Light may affect number recognition process. Default background for written numbers is bright yellow, and pink for frame reference.
- Camera calibration and offset may affect target position of desired number.
- If target position is published wrong from `pixelconvert.py`, Baxter may not find a valid solution since such position are out of workspace
