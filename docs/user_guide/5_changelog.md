# Changelog 

## ROS Melodic

###Version 0.0.20 (Current melodic-release)

* Fixed an issue where the hand Demo did not recognise Demo Hand D had biotacs

### Version 0.0.17

Features:

* Fixed a hand serial issue with launching bimanual hands locally without a NUC

### Version 0.0.16

Features:

* Fixed an issue in Rviz displaying left and right hands in the same location without separation when NUC with external control loop is being used

### Version 0.0.15

Features:

* Fixed an issue in Gazebo9 not displaying the forearms of the hands properly
* Fixed an issue in Rviz displaying left and right hands in the same location without separation

### Version 0.0.14

Features:

* Enabling the bimanual hands only system (no arms) to be run on NUC with external control loop

### Version 0.0.13 (current melodic-release)

Features:

* Fixed deprecation errors for melodic
* Added bimanual with no hands to sr_robot_launch

## ROS Kinetic

### Version 1.0.53 (current kinetic-release)

Features:

* Fixed an issue with Moveit trajectory planning in the Bimanual setup

### Version 1.0.52

Features:

* Fixed a hand serial issue with launching bimanual hands locally without a NUC
* Fixed an issue with launching left or right hand locally without a NUC for ROS Kinetic

### Version 1.0.51

Features:

* Fixed an issue in Rviz displaying left and right hands in the same location without separation when NUC with external control loop is being used

### Version 1.0.50

Features:

* Fixed a bug causing incorrect launch of unimanual left hand in NUC external control loop for ROS kinetic only

### Version 1.0.49

Features:

* Fixed an issue in Rviz displaying left and right hands in the same location without separation

### Version 1.0.48

Features:

* Enabling the bimanual hands only system (no arms) to be run on NUC with external control loop

### Version 1.0.45 (current kinetic-release)

Features:

* Allows Hand control from the NUC
* UR firmware check on docker startup
* New thumb calibration
* Launch files updated

### Version 1.0.38

Features:

* Supports using an external control loop (in a NUC) to launch: hand only, arm only, hand+arm 
* If an arm is connected, there is an automatic arm firmware compatibility check
* Automatic compatibility check of the Docker Image and hand firmwares

### Version 1.0.31

Features:

* Docker image now built in AWS

### Version 1.0.26

Features:

* Added a feature that Docker Image release process checks for pre-existing Docker tags in Dockerhub

### Version 1.0.25

Features:

* Updated launch files
* Added bimanual control
* General bugfixes

### Version 1.0.24

Features:

* Fixing a few bugs with the Data Visualizer
* Hand E Data Visualizer GUI

### Version 1.0.21

Features:

* System logging was added

### Version 1.0.15

Features:

* Moveit warehouse branch was changed to our fork to work well. Official moveit warehouse was crashing

### Version 1.0.12

Features:

* Moved CyberGlove configuration to its own repository. Using the CyberGlove requires the -cg Docker One-liner flag and correct CyberGlove branch to be specified
* If the hand is launched under simulation, use_sim_time is automatically set to true
* Added script to test real-time performance (control loop overruns and signal drops) of the computer running the hand and to specify how many seconds to run for
* Improved ROS save logs functionality by including debug symbols
* Improved ROS save logs functionality by deleting logs over 1 GB (to avoid the computer from filling up)
* Improved ROS save logs functionality (and the upload to AWS) to giving the user the option to decline uploading anything to AWS
* Added CyberGlobe calibration and tweaking plugins to rqt

### Version 1.0.9

Features:

* The Docker container launches in a few seconds

### Version 1.0.7

Features:

* Ability to easily upload ROS Logs to Amazon Web Services (AWS) and email them to Shadow Robot Company automatically
* PyQtGraph used for plotting back-end in rqt

### Version 1.0.5

Features:

* Release of hand E software (kinetic-v1.0.5) and firmware (firmware release 3), using the new firmware release mechanism (GitHub)
* Ability to save ROS logs by clicking on an icon on the desktop

### Version 1.0.2

* Initial version
