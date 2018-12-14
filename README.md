## BaxterReadsNumbers
#### ME 495: Embedded Systems in Robotics
#### _Jordan Haskel, Lukai Jin, Jingyan Ling, James Sohn, Yichi Zhang_


## Introduction

This repo contains ROS packages needed to allow Baxter, manufactured by Rethink Robotics, to perform robust block pick and place using single digit handwritten numbers on the object.  

## Hardware
- Baxter 
- 1080p usb camera
- Yellow 3x3 Post-it (10 ea.)
- Pink 3x3 Post-it (1 ea.)
- Black marker

## Software requirement
- Linux 18.04
- ROS Melodic  

## Highlevel description and included packages/files
### Required package list
- [MoveIt](https://moveit.ros.org/)
- [trac_ik](http://wiki.ros.org/trac_ik)
- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)

### High level description
The functionality of the package consists of two individual packages that are not listed in the required package list.  
`baxterpicknumber` and `number_rec`. Below is the simpe explanation of what each package does.
- `number_rec`: NEEDS JORDAN INPUT
- `baxterpicknumber`: Subscribes to `number_rec` displaying relative position of the object in the external usb camera frame, convert it to position of the object in real world and perform pick and place

### Package breakdown
#### `number_rec`
##### Launch files

##### Nodes

##### Services

#### `baxterpicknumber`  
##### Launch files
`move_joint_target.launch` initiates MoveIt!, Baxterinterface and Rviz

##### Nodes
`move_joint_target.py`: uses [Modern_robotics library ](https://github.com/NxRLab/ModernRobotics) to solve inverse kinematics to maneuver Baxter's right gripper to designated location, grab the object and place it to designated drop off zone.  

`pixelconvert.py`: converts the object location in the usb camera pixel frame into Baxter's world frame. Conversion is done in multiple steps of matrix transformation and tracking of AR tag. Both usb camera and built-in camera in Baxter's left hand track the AR tag providing the relationship between usb camera and Baxter coordinate. Then usb camera locates pik Post-It in the frame to build conversion factor between pixel in the usb camera and the real-world coordinate. The node accepts single-digit number as user input and perform locating, picking up and moving.  

`pixelconvert2.py`: has additional function in the node that decomposes double-digit user input to multiplication of multiple single digit numbers and perform locating, pickng up and moving.  

`move_cartesianpath.py`(optional): is another approach that one can choose to work with instead of `move_joint_target.py` using moveit's cartesianpath computation method.  

`move_motionplann.py`(optional): is another approach to motion plan with moveit's built-in motion planner.  


## Demo video and package run instruction
### Demo video
One can find demo video [here](https://drive.google.com/file/d/18wlpZJT8PQiyQPO6wGeEcdG6VCEspOpR/view)  

### Package run instruction
Listed below is the series of commands to initiate the package and run.
- Connection to Baxter: One can find detailed insturction of workstation setup [here](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)  
- Launch usbcamera and number recognition node   
    `roslaunch baxter_fun numberrecog.launch`  
- Launch coordinate conversion  node  
    `rosrun baxterpicknumber pixelconvert.py`  
- Launch move and pick node  
    `roslaunch baxterpicknumber move_joint_target.launch`    
  

## Issues 
