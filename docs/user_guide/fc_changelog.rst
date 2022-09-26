# Changelog

## ROS Noetic

### Version 1.0.21 (Aurora 2.1.6) - current noetic-release
* Used for release

### Version 1.0.20
* Fixing TF re-publisher to prevent REPEATED_TF messages
* Updated and fixed issue with error reported by bag_rotate node
* Updated bag rotate
* Corrected order of fingers moving when using demo icons

### Version 1.0.19
* Update known good firmware file - UR10 arms
* Fixing rosbag rotate

### Version 1.0.18 (Aurora 2.1.5) - previous noetic-release
* Fixing demo behaviour when tactile sensors are installed

### Version 1.0.17
* Update repository with sr_hand_config
* Fix handling of active rosbags
* Remove default hand serial parameter
* Fixing bug config file pid parameters being erased when saving selected
* Rounding up values for joint slider

### Version 1.0.16
* Fixed linter errors
* Improve realtime publisher fast pid divisor
* Fix joint position/velocity filter
* Fix broken rosbags
* Increase wait for joints_states message timeout on TeachMode
* Fixing arm only launch
* Added a System Health Node
* Removed incorrect error message

### Version 1.0.15
* Improve realtime publisher fast pid divisor
* Fix j0 pos vel filter
* Added getter for hand trajectories
* Fix broken rosbags
* Supporting workspaces without UR components
* Changed default version values when launching hand in simulation
* Fixing teach mode for different hand types
* Fixing arm only launch
* Increasing timeout time in teach mode node
* Removed incorrect error message
* Fixing access modifiers
* Fixed linter errors

### Version 1.0.14 (Aurora 2.1.4) - previous noetic-release
* No changes (release testing image)

### Version 1.0.13
* Adding more time to sleep to reload params
* Added getter for hand trajectories
* Do not require ur_description unless it is needed
* Changed default version values when launching hand in simulation
* Fixing teachmode for different hand types
* Fixing access modifiers

### Version 1.0.12
* Add aws manager test
* Update shadowhands_prefix.srdf.xacro
* Adding "first finger point" to named hand states
* Update arm and hand examples

### Version 1.0.11
* Update demo

### Version 1.0.10
* Fix calibration loader
* Fixing demo for left hand
* Fixed linter errors
* Adding bimanual support to data visualizer

### Version 1.0.9
* No changes (release testing image)

### Version 1.0.8
* No changes (release testing image)

### Version 1.0.7
* Updated AWS Manager to allow for subfolders
* Fix error with decoding git commands in ws_diff
* removed unused imports from sr_ur_arm_calibration_loader.py
* Removing sr_config repo
* Fixing shebang and file saving in Hand Health Report
* fix building error: This package requires sr_visualization_icons to build, and this patch fixes it
* Solve bug Unfiltered position and force traces not shown
* Update warnings in RQT
* Adding serial number to FingertipVisualizer plugin

### Version 1.0.6
* Removed roswrapper from launch files using Autodetect
* Fix missing use namespace EigenCompiling packages for the `ros-o` initiative
* Fixed mistake in file change_controllers.py
* Delete sr_teleop_polhemus_documentation_server.py
* Removed ros files for sr_teleop_polhemus_documentation

### Version 1.0.5
* Changing default vaules of fingertip sensors srhand.launch

### Version 1.0.4
* Xacros refactored
* Remove obsolete scoped_ptr
* Switching to new xacros
* Fixing bugs in launch files
* Adding return to plan executions
* Removing box from arm without hand and bimanual system without hands
* Deleted sr_box_ur10_moveit_config folder
* Refactor robot commander test
* Removing sr_hand_dep
* Removing deprecated field from general_info
* Fix phantom hand
* Removed old launch file with box and replaced with the new one from sr_interface
* Support ImageMagick 6 and 7
* Hand side fix error

### Version 1.0.3
* Migrating to dae and adding materials
* Fixing the color of wrist mesh
* Switching to new xacros
* Update arm related arguments in sr_robot_launch
* Adding a way of exiting the demo
* Edit tactile threshold
* Showing allowed options for general info template
* Re-write data visualizer

### Version 1.0.0 (Aurora 2.0.0) - previous noetic-release
* Integrate UR driver from upstream
* Refactoring sr_description: adapted test and added more parameters validation
* Create trajectory command publisher utility class
* Migrate controls and calibrations
* Fixing wrist controller spawning and updating/cleaning up controller spawner script and docs.
* Add voice feedback to voice controller
* Listen to topics to detect speaker/microphone changes
* Replace PyDub library with a direct call to ffmpeg
* Adding republish tf new place
* Integrate UR driver from upstream
* Updating tf republisher
* Adding collision scene for filling line
* Add hybrid controller argument to more launch files
* Removing external control option for sim
* Removing sr_config references
* Fix robot_commander test in AWS
* Make wrist trajectory controller it's own entity
* Integrate ur driver from upstream
* Fixing scene spawning
* Xacro package changed, now needs a function call to setup file stack for error reporting
* Fixing controllers for hand lite
* Fixing movegroup controller problem
* Fix planning errors
* Fixing wrist controller spawningFixing wrist controller spawning.
* Fix __kinematics
* Loading analyzers from new place
* Migrate controls
* Migrate calibrations
* Loading rates from a new place
* Deprecating sr config
* Migrate controls
* Migrate analyzers
* Migrate calibrations
* Migrate rates
* Fixed the calibration for both lph and rph.
* Integrating auto-detection
* Fixing errors when changing controllers and resetting joint sliders

### Version 0.0.18
* Update rviz_motor.launch
* Fixed Relative path
* Add hybrid controller configuration files
* Load hybrid controller configuration
* Remove redundant aws manager
* Removing hand detector
* Move sr_world_generator from common_resources to sr_tools
* Add world & scene for XPrize competition
* Fixed aws_manager
* Enhancing cond delay tool
* Prepare the piezo driver to work with multiple dev-kits
* simple executable ros wrapper
* fixing the tests
* Integrated autodetection
* Add hybrid controller argument to more launch files
* Removing robot description
* Adding configs for clients in noetic
* Move sr_world_generator from common_resources to sr_tools
* Added missing resource and uis install for sr_data_visualization
* Removing muscle rqt plugins
* Added missing resource and uis install for sr_data_visualization
* Removing grasp controller from plugins

### Version 0.0.17 (Aurora 1.1.8) - previous noetic-release

* Update tactile_receiver.py
* Move conditional delayed rostool to src and add launch prefix for launching nodes
* Load hand trajectory controller for hand in sim use case
* Adding trajectory controllers for bimanual
* B revert wrist in arm controller move group fix

### Version 0.0.16

* Robot commander fix

### Version 0.0.15

* Adding new xacro for a hand extra lite with only two fingers mf and th
* Limiting sim speeds to 1.0, now that CPUs are fast enough.
* Fixed linter error in hpp file
* Dixed linter errors in hpp files

### Version 0.0.12

* Update simple_transmission.hpp
* Revert "SRC-4962 Move controller switching to CPP (#647)"

### Version 0.0.11

* Fixing SrRobotCommander

### Version 0.0.10

* Adding hybrid file
* F#src 6473 handle 0 in git revision
* SRC-6470 Release noetic dexterous hand image
* SRC-4962 Add changes from teach_mode_node
* SRC-6063 Don't busy wait for params
* Changing to correct launchfile
* Adding prefix to ur10e yamls
* F#src 6509 optimise arm unlock noetic
* F#src 6509 optimise arm unlock
* SRC-4962 Use helper class from common_resources
* F#src 6477 sr ur arm unlock test noetic
* SRC-4962 Move controller switching to CPP
* initial commit for mock ur dashboard server
* Adding arm servo noetic
* SRC-6177 Fix little finger error reporting
* Integrating hybrid controller
* fixing noetic
* SRC-6470 Release noetic dexterous hand image
* Fixing bootloader path with casting to string

### Version 0.0.9

* F#src 6509 optimise arm unlock noetic
* F#src 6509 optimise arm unlock
* Fixing bootlo* ader path with casting to string

### Version 0.0.8

* F#src 6473 ha* ndle 0 in git revision
* SRC-6470 Rele* ase noetic dexterous hand image
* Adding prefix to ur10e yamls

### Version 0.0.7

* SRC-6470 Rele* ase noetic dexterous hand image

### Version 0.0.6

* Fixed deprecated .mesh
* F#98 modular * xacros
* SRC-6467 Intr* oduce git_revision field in GenericTactileData
* Update demo_r* .py
* Src 6413 create a collision model for the rack
* add only stan* s
* B fixing watchdog test
* F fixing speech control
* SRC-6470 Release noetic dexterous hand image
* SRC-6301 Implement reading of MST sensors
* Update package.xml

### Version 0.0.5

* fix pedal bug
* B pedal restart fix

## ROS Melodic

### Version 0.0.62  (current melodic-release)

* Improving saving utility for Noetic
* Fixing yaml load
* Adding respawn
* Fixed calibration loader
* Automatic calibration loader not working in URSIM
* Adding missing arguments
* SRC-6043 Remove unused 'rename' arguments
* Adding kill node script
* SRC-5239: Adding speech control
* SRC-6183 Add __init__.py file
* SRC-6183 Various improvements for speech control
* Fixing yaml load
* arms braking
* fix home
* removing the required flags
* Fix_an_arm_and_hand_xacro
* Adding x and y separations to launch and xacros
* changing jiggle fraction default value
* Update sr_ur_arm_unlock
* fix syntax error
* Automatic calibration loader not working in URSIM
* Publish underactuation error
* Fixing srdf generation and saving of file
* Fixing yaml load
* improving hand and arm rostest
* Commenting trac_ik and replacing it to kdl until it is available in Noeticoetic
* updating unimanual y separation
* Fix pedal reset for protective stop
* Add new driver for teleop pedal
* Update 90-VEC-USB-Footpedal.rules

### Version 0.0.61

* Fix pedal reset for protective stop

### Version 0.0.60

* Improving saving utility for Noetic
* Fixing yaml load
* Adding missing arguments
* Remove unused 'rename' arguments
* Adding kill node script
* Adding speech control
* Add __init__.py file
* Various improvements for speech control
* Fixing yaml load
* Publish underactuation error
* Fixing srdf generation and saving of file
* Fixing yaml load
* improving hand and arm rostest
* Commenting trac_ik and replacing it to kdl until it is available in Noeticoetic

### Version 0.0.58

* Changing paramiko version to 2.7.2
* Adding respawn
* Merging kinetic-devel back to melodic
* Fixed calibration loader
* Fixed arm and hand xacro
* Automatic calibration loader not working in URSIM
* Fixing orientation for left arms
* Fixing xacro
* Hand and arm test
* Arms braking
* Fix home
* Removing the required flags
* Updating unimanual y separation
* Adding X and Y separations to launch and xacros
* Changing jiggle fraction default value
* Update sr_ur_arm_unlock
* Fix syntax error
* Fix data visualization bug
* Add new driver for teleop pedal
* Update 90-VEC-USB-Footpedal.rules

### Version 0.0.57 (previous melodic-release)

* Merging kinetic-devel back to melodic
* Fixing orientation for left arms
* Fixing xacro for sr_multi_description/urdf/right_srhand_lite_ur10e.urdf.xacro
* Adding hand and arm tests in robot launch
* Fix data visualization plugin bug

### Version 0.0.56

* Add wait for robot description in sr_robot_launch/launch/sr_ur_arm_box.launch
* Plotjuggler v3

### Version 0.0.55

* Update calibration GUI

### Version 0.0.54

* Fetch arm ips from param server
* fixing set_named_target method in robot commander

### Version 0.0.53

* Fix for hand finder overwriting urdf joints with all joints
* Add default to launch arg list
* Delete pull_request_template.md
* Adding wait to watchdog
* Fixing home angle arg in sr_robot_launch files
* Updating worlds and scenes to bimanual
* Adding the planning group two_hands
* Updating state saver for more options

### Version 0.0.52

* Delete pull_request_template.md
* Fix for hand finder overwriting urdf joints with all joints
* Add default to launch arg list in conditional delay

### Version 0.0.51

* Update sr_bimanual_ur10arms_hands.launch
* Adding start state to stored states
* Update planner to BiTRRT
* Modify parameter to load robot description at this level only if requested

### Version 0.0.50

* Demohand a with ur10e updated

### Version 0.0.49

* Adding hybrid controller arbitrary frame
* Removing exclude wrist from controller spawner
* Removing include_wrist_in_arm_controller param
* Adding planning quality to examples
* Adding scripts and documentation for in-docker leap motion running
* Bimanual demohands a d changes
* wrist mimic rostest
* Fix left arm scene
* add sr_robot_msg dependency

### Version 0.0.48

* Created bimanual xacro for hand lites biotacs

### Version 0.0.47

* Fixed hybrid controller installation and controller spawner
* Tests for the scene

### Version 0.0.46

* Added hybrid controller
* Added a xacro for shadow hand left lite with biotacs
* Fixed install of ros_heartbeat
* Updated aurora instructions to specify ethercat_right_hand rather than ethercat_interface
* Fixed conditional roslaunch (added extra conditions)
* Adding and updating hand ROS tests
* New scene and world for MS lab 
* add cpp wait for param
* updating open hand demo for smoother opening 

### Version 0.0.45

* Added stand to simulation
* Updated README
* adding additional check

### Version 0.0.44 (previous melodic-release)

* Created /run/user/1000 folder inside the docker container (to fix rqt graphics issue)

### Version 0.0.43

* Local hw interface and fixed do switch with centre of gravity

### Version 0.0.42

* Updated README.md

### Version 0.0.41

* Fixed and added files to make the ur5e with box work and generify the launch file
* Added metapackage

### Version 0.0.40

* Updated sr_system.launch
* Added full hand ur5e support
* Added ur5e normal hand configs

### Version 0.0.39

* Shadow glove GUI updated and moved

### Version 0.0.38

Features:

* Updated calibration GUI

### Version 0.0.37

Features:

* Tone down UR10e tuning so the arm behaves more smoothly

### Version 0.0.35

Features:

* Fix hand control parameter error in setting the payload for UR arm

### Version 0.0.34

Features:

* Update motor effort file for left hand
* Add relay node with tcp_nodelay param
* Hand + UR arm: allow setting cog and payload
* Use Shadow's fork of universal robot repositor
* Fix biotac visualizer for bimanual
* change yaw roll, adjust formulas after real hw testing
* Fix sensor manager file 

### Version 0.0.33

Features:

* Changing expected delimiter from newline to '_' in arm firmware checker
* Adding x and y separation for left bimanual arm config

### Version 0.0.32

Features:

* Set arm IP defaults to new values (10.8.1.1 and 10.8.2.1) and also added a comment about aurora using sed to replace these IPs
* Changed hand mapping path default to v4
* fix for arm in safety violation mode
* second try at adding ur10 config, minimal changes
* Fixing controller spawning bug in which WRJ1+2 would not work when wrist was included in arm trajectory control 
* Fixing controller spawning bug in which WRJ1+2 would not work when wr 
* Updating calibration gui 

### Version 0.0.31

Features:

* Fixed bug in Dexterity Test that stopped hand moving to the correct poses.
* Fixed bug in the Bimanual launch files to load correct planning groups.
* Mujoco ur hand
* Fix ur box
* Fixing bug wherein conditional delay script would count found parameter
* Adding gui for shadow glove calibration
* Moving hand meshes to a more standard path to make gzweb work
* parsing hand sides
* remove user choice, add conditional delay
* arm calibration loader 2
* Adding wrapper script for autodetecting shadow hands

### Version 0.0.30

Features:

* Fixed bug in RQT Data Visualiser that stopped other plugins from plotting

### Version 0.0.29

Features:

* Config and xacro for hand lite ur10e
* Fixed bug with ur_arm_release
* Fixed conditional delay bug in sr_interface

### Version 0.0.28

Features:

* now correctly handles exception
* config and xacro for hand lite ur10e
* Adding support for ur5e and hand lite
* fixing error message

### Version 0.0.27

Features:

* adding hand mapping v4 files
* enable ft sensor on ur e robots
* adding la_ur10e_with_box xacro
* fixed sr_hardware control loop bug
* Adding scene and world for ms garage 
* Update sr_ur10arm_box.launch 
* adding mapping v4
* Fixing args being limited to group scope
* Restoring arm_ and hand_ctrl control loop arguments to the previous f
* Adding mock triple pedal
* Fixing intermittent bug in controller spawning
* Updating real time TF republisher for more flexibility
* adding ur10e with box yaml files

### Version 0.0.26

Features:

* Updated controller spawner
* Replaced delay roslaunch with conditional roslaunch


### Version 0.0.24

Features:

* Fixed an issue where the config files did not contain a robot_config_file parameter, preventing launch
* Fixed an issue where robot_description was not found for the NUC setup
* Fixed an issue preventing the effort controllers to launch

### Version 0.0.20

Features:

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

### Version 0.0.13

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

* Release of hand E software (kinetic-v1.0.5) and firmware (firmware release 3), using the new firmware release mechanism
* Ability to save ROS logs by clicking on an icon on the desktop

### Version 1.0.2

* Initial version
