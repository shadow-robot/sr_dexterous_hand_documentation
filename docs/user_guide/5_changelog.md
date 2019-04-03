# Changelog 
## ROS Kinetic

### Version 1.0.24 (current kinetic-release)

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
