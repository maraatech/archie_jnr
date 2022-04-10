# mini_rig_ros
Packages to operate the Mini Rig platform.
Mini Rig is a simple base frame built by UoW to mount a UR5 robotic arm. 
This package contains all the urdf and description files required to opreate the Mini Rig with the UR5 with a number of possible sensors attached.
The control of the Mini Rig platform defaults to ur_moveit_controller but this can be swapped for other means of moving the arm if required.

Insert Picture Here

## Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.
See deployment for notes on how to deploy the project on a live system.

### Prerequisites
Base package dependencies

```
1) Ubuntu 20.04 - ROS Noetic - requires python 3 support

2) Pull master version of cares_msgs
   a) cd ~/catkin_ws/src
   b) git clone https://github.com/UoA-CARES/cares_msgs.git
```

### Installing
A step by step guide on how to get mini-rig up and running.

#### Universial Robot Arm Packages
Pull down packages related to the UR5 arm.
Follow instructions as laid out under "Building" here: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.
NOTE: We are using catkin_ws so do not create a new one - start with "clone the driver" into catkin_ws/src

If the arm has not been used before then follow instructions on extracting the calibration file (note: this has been done for the UoA and UoW UR5 already).
The arm calibration is placed under the location mini_rig_ros/mini_rig_description/config.

#### Control Package
The controller for the Mini Rig platform is the simple ur_moveit_controller.

Install the ur_moveit_controller: https://github.com/maraatech/ur_moveit_controller

#### Sensor Packages
Depending on which sensor/s configuration you are using follow the corrosponding sensor install instructions:

Stereo Cameras
Pylon Cameras: https://github.com/UoA-CARES/pylon_camera
Basler Cameras: 

Zivid:
TBC

Realsense:
TBC

#### Install Mini Rig Packages
Clone the package into the catkin directory you are using, presumed here to be "~/catkin_ws"

```
cd ~/catkin_ws/src
git clone https://github.com/maraatech/mini_rig_ros.git
```

Build the package with catkin_make in the source directory

```
cd ~/catkin_src/
catkin_make
```

### Calibrating Mini Rig
The Mini Rig requires calibration in order to align the sensor and arm before it can be used for other tasks.

Install the Stereo Hand Eye Calibration package: https://github.com/UoA-CARES/cares_hand_eye_calibration

Stereo and Hand eye calibration is automated for the mini-rig platform. 
To conduct the calibration run the follow launch files below.
The calibration target and other parameters can be adjusted within these launch files.
DO NOT ADJUST THE LAUNCH FILES IN cares_hand_eye_calibration.

```
#Brings up the mini_rig platform
roslaunch mini_rig_bringup rig_bringup.launch

#Run the stereo calibration
roslaunch mini_rig_bringup rig_stereo_calibrate.launch
```

The calibration files will be saved under ~/calibration_images/YYYY_MM_DD_HH_MM_SS as calibration.json (stereo calibration) and stereo_handeye.yaml (stereo_handeye calibration).
If you are happy with the calibrations place the calibration.json under pylon_camera/config and stereo_handeye.yaml under mini_rig_ros/mini_rig_description/config.

