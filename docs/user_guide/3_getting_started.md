# Getting started

## First time users
If you are unfamiliar with ROS and intend to use the ROS API, it is highly recommended that you read the [ROS Tutorials](http://www.ros.org/wiki/ROS/Tutorials).

If you are unfamiliar with the terminal on Linux, you should look [here](https://askubuntu.com/questions/183775/how-do-i-open-a-terminal).

Shadow software is deployed using Docker. Docker is a container framework where each container image is a lightweight, stand-alone, executable package that includes everything needed to run it. It is similar to a virtual machine but with much less overhead. Follow the instructions in the next section to get the latest Docker container of the hand driver and interface up and running.

## Hardware specifications

In order to run our software and the ROS software stack you will need to meet some hardware requirements.

CPU: Intel i5 or above
RAM: 4GB or above
Hard Drive: Fast HDD or SSD (Laptop HDD are very slow)
Graphics Card: Nvidia GPU (optional)
LAN: A spare LAN port to connect the Hand (even with a USB to LAN adaptor)
OS: Ubuntu 18.04, 16.04 Kinetic (Active development) or 14.04 Indigo for older releases.

The most important one is to have a fast HDD or an SSD.

## Docker
### Installing the software on a new PC using the one-liner
We have created a one-liner that is able to install Docker, download the image and create a new container for you. It will also create two desktop icons, one to start the container and launch the hand and another one to save the log files locally. To use it, you first need to have a PC with Ubuntu installed on it (preferable version 16.04) then follow these steps:

#### Installing for a real robot

* **Check your hand interface ID**:

  Before setting up the docker container, the EtherCAT interface ID for the hand needs to be discovered. In order to do so, after plugging the hand’s ethernet cable into your machine and powering it up, please run

  ```bash
  $ sudo dmesg
  ```
  command in the console. At the bottom, there will be information similar to the one below:

  ```bash
  [490.757853] IPv6: ADDRCONF(NETDEV_CHANGE): enp0s25: link becomes ready
  ```
  In the above example, ‘enp0s25’ is the interface ID that is needed.

* **Get ROS Upload login credentials**

  If you want to upload technical logged data (ROS logs, backtraces, crash dumps etc.) to our server and notify the Shadow's software team to investigate your bug then you need to enable logs uploading in the one-liner. In order to use this option you need to obtain a unique upload key by emailing sysadmin@shadowrobot.com. When you receive the key you can use it when running the one-liner installation tool. To enable the logs uploading you need to add the command line option ```-ck true``` to the one-liner.
  After executing the one-liner, it will prompt you to enter your upload key and press enter to continue. Please copy and paste your key from the email you received by Shadow Robot.

* **Check your hand configuration branch**:

  You should have the name of your [sr_config](https://github.com/shadow-robot/sr-config) hand branch which contains the specific configuration of your hand (calibration, controller tuning etc…).
  Usually it is something like this: ``shadowrobot_XXXXX``.

  If you are unsure please contact us.

* **Run the one-liner**:

  The one-liner will install Docker, pull the image from Docker Hub, and create and run a container with the parameters specified. In order to use it, use the following command:

  **Please remember to replace [EtherCAT interface ID] with your Interface ID and [sr_config_branch] with your unique sr_config branch**

  ROS Kinetic (Recommended):
  ```bash
  $ bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:kinetic-release -n dexterous-hand -sn Hand_Launcher -e [EtherCAT interface ID] -b [sr_config_branch]
  ```
  Examples:
  For Interface ID ```ens0s25``` and sr_config_branch ```shadow_12345```
  ```bash
  $ bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:kinetic-release -n dexterous-hand -sn Hand_Launcher -e ens0s25 -b shadow_12345
  ```  
  Same as above but with ROS logs upload enabled
  ```bash
  $ bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:kinetic-release -n dexterous-hand -sn Hand_Launcher -e ens0s25 -b shadow_12345 -ck true
  ```  

  ROS Indigo:
  ```bash
  $ bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:indigo-release -n dexterous-hand -sn Hand_Launcher -e [EtherCAT interface ID] -b [sr_config_branch]
  ```
  Examples:
  For Interface ID ```ens0s25``` and sr_config_branch ```shadow_12345```
  ```bash
  $ bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:indigo-release -n dexterous-hand -sn Hand_Launcher -e ens0s25 -b shadow_12345
  ```  
  Same as above but with ROS logs upload enabled
  ```bash
  $ bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:indigo-release -n dexterous-hand -sn Hand_Launcher -e ens0s25 -b shadow_12345 -ck true
  ```  

  You can also add -r true in case you want to reinstall the docker image and container. When it finishes it will show:
  ```bash
  Operation completed
  ```
  and it will create two desktop icons on your desktop that you can double-click to launch the hand or save the log files from the active containers to your desktop.
  The icon that launches the hand looks like this:

  ![desktop_icon](../img/desktop_icon.png)

  And for saving the logs:

  ![log_icon](../img/log_icon.png)

#### Installing for using it in simulation

If you do not actually have a real hand but would like to use our hand in simulation, then please run the following command:

ROS Kinetic (Recommended):
```bash
$ bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:kinetic-release -n dexterous-hand -sn Hand_Launcher -b kinetic_devel -l false
```

ROS Indigo:
```bash
$ bash <(curl -Ls http://bit.ly/launch-sh) -i shadowrobot/dexterous-hand:indigo-release -n dexterous-hand -sn Hand_Launcher -b kinetic_devel -l false
```

You can also add -r true in case you want to reinstall the docker image and container. When it finishes it will show:

```bash
Operation completed
```
and it will create two desktop icons on your desktop that you can double-click to launch the hand or save the log files from the active containers to your desktop.

### Using a PC that Shadow provided
In this case, the previous steps would have been performed by the Shadow team before, then the only thing to do to start the Hand is to either double-click the desktop icon or to run the container using:

```bash
$ docker start dexterous-hand
```

You can check the currently available containers using:
```bash
$ docker ps -a
```

The container will be ready when fingers move to the zero position.

## Saving log files and uploading data to our server
When running the one-liner, along with the icon that starts the Grasper, you will also notice a second icon named Save logs that is used to retrieve and copy all the available logs files from the active containers locally on your Desktop. This icon will create a folder that matches the active container's name and the next level will include the date and timestamp it was executed. When it starts, it will prompt you if you want to continue, as by pressing yes it will close all active containers. If typed 'y' to continue, you will have to enter a description of the logging event and will start coping the bag files, logs and configuration files from the container and then exit. Otherwise, the window will close and no further action will happen. If you provided an upload key with the one-liner installation then the script will also upload your LOGS in compressed format to our server and notify the Shadow's software team about the upload. This will allow the team to fully investigate your issue and provide support where needed.

## Starting the driver (Real hand)

### Shadow Hand Driver
  Launch the driver for the Shadow Hand using the desktop icon 'Hand_Launcher' or at a
  terminal (in the container), type:

  ```bash
  $ sudo -s
  $ roslaunch sr_ethercat_hand_config sr_rhand.launch
  ```

  **Warning**: This terminal now has root privileges, and the system is giving you
  permission to do things which can mess up the configuration of the PC. Do not run any
  other commands in this terminal window. Only use this terminal to launch the driver.

### Lights in the hand:
  When the ROS driver is running you should see the following lights on the Palm:

  ```eval_rst
  ========================   =============       ================    =================================
  Light                      Colour              Activity            Meaning
  ========================   =============       ================    =================================
  Run                        Green               On                  Hand is in Operational state
  CAN1/2 Transmit            Blue                V.fast flicker      Demand values are being sent to the motors
  CAN1/2 Receive             Blue                V.fast flicker      Motors are sending sensor data
  Joint sensor chip select   Yellow              On                  Sensors being sampled
  ========================   =============       ================    =================================
  ```

  After killing the driver, the lights will be in a new state:
  ```eval_rst
  ========================   =============       ================    =================================
  Light                      Colour              Activity            Meaning
  ========================   =============       ================    =================================
  Run                        Green               Blinking            Hand is in Pre-Operational state
  CAN1/2 Transmit            Blue                Off                 No messages transmitted on CAN 1/2
  CAN1/2 Receive             Blue                Off                 No messages received on CAN 1/2
  Joint sensor chip select   Yellow              Off                 Sensors not being sampled
  ========================   =============       ================    =================================
  ```

## Graphical User Interface

The majority of functionality is provided by the software Application Programmer Interface (API). However, a few simple functions are provided in the Graphical User Interface (GUI) to test the hand, validate that it is working correctly, and adjust some of its settings.

### Starting the interface
You may open the Graphical User Interface to try out some functions of the hand. From the Docker terminal, type:
```bash
$ rqt
```

  This interface contains a number of plugins for interacting with the EtherCAT hand. Most of them are available from the **Plugins → Shadow Robot** menu.

### Robot Monitor
We can check that everything on the robot is working correctly using the Diagnostic Viewer.

  **Plugins → Robot Tools → Diagnotic Viewer**

  ```eval_rst
  .. image:: ../img/robot_monitor.png
  ```
  Robot Monitor window


This brings up a dialog box containing a tree of all parts of the robot. All parts should be marked with a green tick.

You can examine one motor in detail by double-clicking on it. This brings up the Motor Monitor dialog. This window can be used to check the status of a motor, or debug any problems.

```eval_rst
.. image:: ../img/monitor_single_motor.png
```
Monitoring a single motor

The following table has some more information on what each of these fields mean.

<!--
```eval_rst
============================   ==============================================================================================
Item                           Description
============================   ==============================================================================================
Full Name
Component
Hardware ID
Level
Message                        Any error or status messages
Motor ID                       This is the motor number. Range [0..19]
Motor ID in message            For debugging only
Strain Gauge Left / Right      These are the ADC readings from the two gauges
Executed Effort
Motor Flags                    See motor flags table below
Measured current               Current flowing through the motor (Amps)
Measured Voltage               The motor power supply voltage. Not the voltage at the motor
Temperature                    The temperature measured near the motor. The actual motor winding temperature will be higher than this. (ºC)
Number of CAN messages         Received messages should be twice the transmitted messages
Force control P, I, D terms    These are the PID terms from inside the motor's torque controller. They may be useful for debugging if plotted.
Force control F, P, I, D,
Imax, Deadband, Sign           These are the FPID gain settings used by the motor's torque controller. They can be changed using the controller tuner.
Last Measured Effort           Difference between the two gauge readings (Torque)
Last Commanded Effort          Torque requested by the host-side control algorithms
Encoder Position               The angle of the joint in radians (ROS always calls this Encoder position, even if the robot uses Hall effect sensors)
Firmware svn revision          xxxx: The latest version of the firmware available at build time
                               xxxx: The version of the firmware in the motor MCU
                               False: There are no un-checked-in modifications to this firmware. This should never be true.
============================   ==============================================================================================
``` -->


```eval_rst
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
|                   Item                         |                                                       Description                                                       |
+================================================+=========================================================================================================================+
| Full Name                                      |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Component                                      |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Hardware ID                                    |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Level                                          |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Message                                        | Any error or status messages                                                                                            |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Motor ID                                       | This is the motor number. Range [0..19]                                                                                 |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Motor ID in message                            | For debugging only                                                                                                      |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Strain Gauge Left / Right                      | These are the ADC readings from the two gauges                                                                          |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Executed Effort                                |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Motor Flags                                    | See motor flags table below                                                                                             |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Measured current                               | Current flowing through the motor (Amps)                                                                                |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Measured Voltage                               | The motor power supply voltage. Not the voltage at the motor                                                            |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Temperature                                    | The temperature measured near the motor. The actual motor winding temperature will be higher than this. (ºC)            |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Number of CAN messages                         | Received messages should be twice the transmitted messages                                                              |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Force control P, I, D terms                    | These are the PID terms from inside the motor's torque controller. They may be useful for debugging if plotted.         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Force control F, P, I, D, Imax, Deadband, Sign | These are the FPID gain settings used by the motor's torque controller. They can be changed using the controller tuner. |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Last Measured Effort                           | Difference between the two gauge readings (Torque)                                                                      |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Last Commanded Effort                          | Torque requested by the host-side control algorithms                                                                    |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Encoder Position                               | The angle of the joint in radians (ROS always calls this Encoder position, even if the robot uses Hall effect sensors)  |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Firmware svn revision                          | xxxx: The latest version of the firmware available at build time                                                        |
+------------------------------------------------|                                                                                                                         |
|                                                | xxxx: The version of the firmware in the motor MCU                                                                      |
+------------------------------------------------|                                                                                                                         |
|                                                | False: There are no un-checked-in modifications to this firmware. This should never be true.                            |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
```

```eval_rst
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
|                   Item                         |                                                       Description                                                       |
+================================================+=========================================================================================================================+
| Full Name                                      |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Component                                      |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Hardware ID                                    |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Level                                          |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Message                                        | Any error or status messages                                                                                            |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Motor ID                                       | This is the motor number. Range [0..19]                                                                                 |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Motor ID in message                            | For debugging only                                                                                                      |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Strain Gauge Left / Right                      | These are the ADC readings from the two gauges                                                                          |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Executed Effort                                |                                                                                                                         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Motor Flags                                    | See motor flags table below                                                                                             |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Measured current                               | Current flowing through the motor (Amps)                                                                                |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Measured Voltage                               | The motor power supply voltage. Not the voltage at the motor                                                            |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Temperature                                    | The temperature measured near the motor. The actual motor winding temperature will be higher than this. (ºC)            |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Number of CAN messages                         | Received messages should be twice the transmitted messages                                                              |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Force control P, I, D terms                    | These are the PID terms from inside the motor's torque controller. They may be useful for debugging if plotted.         |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Force control F, P, I, D, Imax, Deadband, Sign | These are the FPID gain settings used by the motor's torque controller. They can be changed using the controller tuner. |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Last Measured Effort                           | Difference between the two gauge readings (Torque)                                                                      |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Last Commanded Effort                          | Torque requested by the host-side control algorithms                                                                    |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Encoder Position                               | The angle of the joint in radians (ROS always calls this Encoder position, even if the robot uses Hall effect sensors)  |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
| Firmware svn revision                          | xxxx: The latest version of the firmware available at build time                                                        |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
|                                                | xxxx: The version of the firmware in the motor MCU                                                                      |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
|                                                | False: There are no un-checked-in modifications to this firmware. This should never be true.                            |
+------------------------------------------------+-------------------------------------------------------------------------------------------------------------------------+
```

### Controller tuner
It is possible to adjust the settings for any of the Position or Force (Motor) controllers.
	**Plugins → Shadow Robot → Basic → Controller Tuner**
#### Position controller
```eval_rst
.. image:: ../img/adjust_position_controller.png
```
  Adjusting the position controller settings



  Here you can select a finger, thumb or wrist joints, and adjust the different position control parameters. Click ```Set Selected``` to send the new values to the motors and make them take effect.

* **“P”, “I” & “D” terms:**  Gain parameters of the position PID controller. By default, Shadow tunes the paramenters using P or PD combinations. The user can add “I” gains in the control if they consider it necessary.

* **Max_force:** This puts a limit on the output (PWM) value that will be sent from the host to the motor by the position controller. It can be useful when setting up a controller for the first time to limit the motor power to a safe level.

* **Position_Deadband:** The error is considered to be zero if it is within ±deadband. This value should be set as a little more than the noise on the sensor. The units of deadband are the same as the value being controlled. So, the deadband for a position controller is in radians.

#### Force controller
```eval_rst
.. image:: ../img/adjust_torque_controller.png
```
Adjusting the torque controller settings

* **“P”, “I” & “D” terms:** Gain parameters of the torque PID controller. By default, Shadow tunes the paramenters using just P gain for the torque control.

* **Max_PWM:** This puts a limit on the final PMW value that will be sent to the motor by the torque controller. It can be useful when setting up a controller for the first time to limit the motor power to a safe level.

* **Deadband:** The error is considered to be zero if it is within ±deadband. This value should be set as a little more than the noise on the sensor. The units of deadband are the same as the value being controlled. The deadband for a torgue controller is in the units of the strain gauges.

* **Torque_Limit:** This value is used to limit the PWM at the end of the control loop. The control algoritm reduces the final PWM that goes to the motor making sure that the force in the strain gauge doesn’t overcome this limit value.

Click ```Save``` to save your settings.

### Bootloader
The firmware in the motors MCUs can be updated from the PC, without opening up the motor base. This can be done from the GUI. Shadow will send you a new HEX if there is an update.
	**Plugins → Shadow Robot → Advanced → Motor Bootloader**

You will see a window listing each motor board, along with its current firmware SVN revision number.

```eval_rst
.. image:: ../img/bootloading_new_firmware.png
```
Bootloading new firmware in to the motor microcontrollers

* **Select Bootloader Hex File:** Next, tell the plugin which firmware to use. The file you should choose here is the one sent by Shadow.

* **Select your motors:** Now you may choose which motors to program. Either select one or more motors using the tick boxes, or click the ```Select All``` or ```Deselect All``` button.

* **Program Motors:** Now you can click the ```Bootload Motors``` button. The process is fairly slow, and takes about a 30 second per motor.

### Change controllers
Use the *Change Controllers* plugin to load one of the three different types of controllers set by default. Simply click on a controller type, and it will call a service from the pr2_controller_manager to unload the currently running controller if necessary, and load the one you've selected.
	**Plugins → Shadow Robot → Change Controllers**

  ```eval_rst
  .. image:: ../img/selecting_different_control_mode.png
  ```
Selecting different control mode

### Advanced controllers
Apart from the three standard controls, you can set the parameters for different control strategies (host – motor) from this plugin.
	**Plugins → Shadow Robot → Advanced → Advanced Controls**

  ```eval_rst
  .. image:: ../img/selecting_different_control_strategy.png
  ```
  Selecting different control strategies

  **NOTE: CURRENTLY THE ONLY FULLY SUPPORTED TYPES ARE POSITION - PWM CONTROL** *(position control),* **AND EFFORT - TORQUE CONTROL** *(teach mode control).* **SELECTING OTHER TYPES MAY CAUSE UNPREDICTABLE RESULTS AND DAMAGE THE HARDWARE.**

  ### Motor Resetter
If for some reason you need to reset the firmware on a motor, you can either press the reset button on the PCB itself (which requires removal of the base covers), or use this plugin.
	**Plugins → Shadow Robot → Basic → Motor Resetter**

  ```eval_rst
  .. image:: ../img/resetting_motor_microcontrollers.png
  ```
  Resetting the motor microcontrollers

  Tick the motors you wish to reset, and click ```Reset Motors```. You should see the corresponding joints jiggle as the motors auto-zero the strain gauges.


### Joint Sliders
A simple interface has been provided to control the position of each joint using a slider (you have to start the position control first).
	**Plugins → Shadow Robot → Joint Sliders**

  ```eval_rst
  .. image:: ../img/joint_sliders.png
  ```
  Joint sliders



  A window with twenty sliders will appear. Moving any slider will cause the corresponding joint on the hand to move.

### Hand Calibration
This plugin is used internally by Shadow to calibrate the raw data from the position sensors.
	**Plugins → Shadow Robot → Basic → Shadow Hand Calibration**

  ```eval_rst
  .. image:: ../img/calibrating_joint_sensors.png
  ```
  Calibrating the joint sensors

  It’s very unlikely that the sensors moved inside of the hand, BUT, if you find misalligments with the model and you require a re-calibration, contact Shadow Robot Company here: <support@shadowrobot.es>.


## Command line interface
All functions of the hand are available from the command line.
### Using rostopic

You can find all the information about the topic published in the Shadow Hand from this link:

<https://shadowrobot.atlassian.net/wiki/spaces/HANDEG/pages/63569986/Hand+E+ROS+Kinetic+Topics>

Assume that all the topics are read only unless specified otherwise.

Definitions used :

Hand = Humanoid Hand (Hand E)

Host = Host Computer, which is controlling the Hand

Contents:

Using rostopic

Trajectory Control

Moveit! Topics

RViz Topics


### Using rostopic
To see at what rate a topic is published use:
```bash
$ rostopic hz <ROS_TOPIC>
```

To see which nodes are publishing and subscribing to topics, as well as the topic message type use:
```bash
$ rostopic info <ROS_TOPIC>
```
Where <ROS_TOPIC> is the topic under scrutiny.

For additional information on ROS topics see : http://wiki.ros.org/rostopic

### Trajectory Control
The following topics described are active using a real Hand E and launching :
```bash
$ roslaunch sr_ethercat_hand_config sr_rhand.launch
```

This rqt_graph shows the flow of topics between nodes whilst running : https://drive.google.com/file/d/1qql0WbgprA80IwDrDELh8RsrF1o3i266/view?usp=sharing


These topics are used during the Hand startup routine to make sure that the Hand is calibrated:

      /cal_sh_rh_*/calibrated
      /calibrated




An empty message is published to the /cal_sh_rh_*/calibrated topics for each joint when they are calibrated. The */calibrate_sr_edc* node subscribes to these topics and when all of them have had a empty message published to them, it publishes True to the /calibrated topic. Before empty messages have been received by all the joints it publishes False to the /calibrated topic.


/diagnostics
/diagnostics_agg
/diagnostics_toplevel_state

These topics update at 2 Hz with information on each joints Temperature, Current, Measured effort and Command effort, as well as information about the EtherCat devices and firmware version, and contain all the diagnostics information that gets published from the fh_driver and fh_safety_checks nodes.

It should not be necessary to publish to these topic from a terminal.

/diagnostics is uncategorized, where the /diagnostics_agg is categorizes using the diagnostic_analyzer.yaml : https://github.com/shadow-robot/fh_config/blob/kinetic-devel/fh_config/diagnostic_analyzer.yaml

You can see the output from these topics in rqt : Plugins->Robot Tools->Diagnostics Viewer :


rqt_diagnostics_gui.png


/joint_0s/joint_states

This topic is not currently used and may be soon removed.


/joint_states

This topic is read-only and updates at 100 Hz with the name, position, velocity and effort values of all joints in a Hand.

Example topic message :

name: [rh_FFJ1, rh_FFJ2, rh_FFJ3, rh_FFJ4, rh_LFJ1, rh_LFJ2, rh_LFJ3, rh_LFJ4, rh_LFJ5,
rh_MFJ1, rh_MFJ2, rh_MFJ3, rh_MFJ4, rh_RFJ1, rh_RFJ2, rh_RFJ3, rh_RFJ4, rh_THJ1,
rh_THJ2, rh_THJ3, rh_THJ4, rh_THJ5, rh_WRJ1, rh_WRJ2]
position: [1.279751244673038, 1.7231505348398373, 1.2957917583498741, -0.00406710173435502, 0.054689233814909366, 1.253488840949725, 1.5395435039130654, 0.02170017906073821, 0.1489674305718295, 1.08814400717011, 1.638917596069165, 1.4315445985097324, 0.00989364236002074, 1.2257618075487349, 1.8331224739256338, 1.2888368284819698, -0.13269012433948385, 0.14435534682895756, 0.6980816915624072, 0.18782898954368935, 1.124295322901818, 0.21905854304869088, -0.048455186771971595, -0.0032803323337213066]
velocity: [-7.484333985952662e-06, -7.484333985952662e-06, 0.0023735860019749185, 0.00062181267775619, -0.0005871136552505063, -0.0005871136552505063, 0.0020967687295392933, 0.0001739028157522596, 0.0004985252400775274, -9.485516545601461e-06, -9.485516545601461e-06, -0.0007068752456452666, -0.0012475428276090576, 0.0008426052935621657, 0.0008426052935621657, 0.001237001167977189, -0.0026444893567459573, 0.0025260047430310925, -0.0003217106977882921, 6.159570145597239e-05, -0.0023454723015513593, 0.0009436399232442155, 0.00017469681801687975, -4.900148416020751e-05]
effort: [-1.3660655058510802, -1.3660655058510802, -2.030169817308198, -1.9577332816789155, 0.0, 0.0, -17.29928766980003, -1.5006516553524243, -1.8579749510438912, -1.504877130092884, -1.504877130092884, -0.3374653182042338, -1.6492254479379729, -8.476660697182016, -8.476660697182016, -3.3867013328219056, -2.3404145772688683, -0.7688013735971971, 11.02319645071454, 0.8482082620071664, 0.08818910881575533, 1.127772119947565, -2.2344970991165316, -3.5544023107705667]


/rh/biotac_*

These topics are read-only and update at 100 Hz with data from the biotac sensors, which comprises their pressure, temperature and electrode resistance. This topic is published from the /biotac_republisher node which receives this data from the driver via the /rh/tactile topic. For further information about the biotacts, refer to their documentation : https://www.syntouchinc.com/wp-content/uploads/2016/12/BioTac_SP_Product_Manual.pdf

Example topic message :

pac0: 2056
pac1: 2043
pdc: 2543
tac: 2020
tdc: 2454
electrodes: [2512, 3062, 2404, 2960, 2902, 2382, 2984, 138, 2532, 2422, 2809, 3167, 2579, 2950, 2928, 2269, 2966, 981, 2374, 2532, 3199, 3152, 3155, 3033]


/rh/debug_etherCAT_data

This topic is published by the driver and updates at 800 Hz with data from the Hand as it is received over EtherCAT, which is useful for debugging.

sensors are the position sensors in the joints, which are included in every packet.

tactile is the data from the tactile sensors, which are included in every packet.

Data is recieved in two alternative packets for the motor torques, each holds data for half of the 20 motors. which_motors states whether is is 0, the first 10 motors, or 1, the other 10 motors.

motor_data_packet_torque is the raw difference between the strain gauge in tension and the strain gauge in compression for each motor.

motor_data_type is used to specify the data in motor_data_packet_misc. This data has been requested from the host. What value corresponds to which data is defined here : https://github.com/shadow-robot/hand-firmware/blob/ff95fa8fc50a372c37f5fedcc5b916f4d5c4afe2/PIC32/nodes/0220_palm_edc/0220_palm_edc_ethercat_protocol.h#L88

which_motor_data_arrived is a bitmap, 20x1 demensional array for the 20 motors, which shows which motors data has been recieved from. For example 349525 = 01010101010101010101.

which_motor_data_had_errors is a bitmap for the motors which have errors.

The tactile sensors attached to the Hand are selected during startup : https://github.com/shadow-robot/hand-firmware/blob/ff95fa8fc50a372c37f5fedcc5b916f4d5c4afe2/PIC32/nodes/common/tactile_edc_ethercat_protocol.h#L74

tactile_data_type is used to specify the data in tactile, similar to motor_data_type and motor_data_packet_misc. In the Example topic message below the PST fingertip sensors are used, its value is refered here : https://github.com/shadow-robot/hand-firmware/blob/ff95fa8fc50a372c37f5fedcc5b916f4d5c4afe2/PIC32/nodes/common/tactile_edc_ethercat_protocol.h#L93

tactile_data_valid is a bitmap for the 5 sensors that is 1 when there are no errors.

There is more data that is recieved from the Hand for the tactile sensors than shown in tactile but it is not published by default.

idle_time_us is the time margin once the Hand has completed its processing and is ready for to communicate on the EtherCAT bus.

Example topic message :

header:
  seq: 176798
  stamp:
    secs: 1528812878
    nsecs: 323410491
  frame_id: ''
sensors: [1303, 1574, 3205, 1780, 1382, 1523, 3164, 1938, 904, 1332, 2977, 1706, 1730, 1434, 3060, 1853, 1955, 1814, 2132, 2294, 2496, 4029, 1668, 2931, 1768, 1377, 26, 27, 28, 29, 30, 31, 0, 19, 8, 9, 0]
motor_data_type:
  data: 3
which_motors: 0
which_motor_data_arrived: 349525
which_motor_data_had_errors: 0
motor_data_packet_torque: [15, -31, -4, 3, 0, 0, -207, -3, -55, -3]
motor_data_packet_misc: [-105, -47, 0, -39, 0, 0, 120, 0, 79, 0]
tactile_data_type: 0
tactile_data_valid: 31
tactile: [407, 429, 416, 398, 389]
idle_time_us: 430
---
header:
  seq: 176799
  stamp:
    secs: 1528812878
    nsecs: 324399217
  frame_id: ''
sensors: [1303, 1574, 3205, 1780, 1382, 1523, 3164, 1938, 904, 1332, 2977, 1706, 1731, 1434, 3060, 1853, 1955, 1814, 2131, 2294, 2496, 4030, 1669, 2931, 1768, 1376, 26, 27, 28, 29, 30, 31, 19, 10, 0, 0, 0]
motor_data_type:
  data: 4
which_motors: 1
which_motor_data_arrived: 699050
which_motor_data_had_errors: 0
motor_data_packet_torque: [-29, -3, 1, -35, -1, -22, -18, 35, 4, 5]
motor_data_packet_misc: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
tactile_data_type: 0
tactile_data_valid: 0
tactile: [407, 429, 416, 398, 389]
idle_time_us: 394


/rh/palm_extras

This topic updates at 84 Hz with data from additional devices plugged into the palm.

Example topic message :

layout:
  dim:
    -
      label: "accelerometer"
      size: 3
      stride: 0
    -
      label: "gyrometer"
      size: 3
      stride: 0
    -
      label: "analog_inputs"
      size: 4
      stride: 0
  data_offset: 0
data: [26.0, 27.0, 28.0, 29.0, 30.0, 31.0, 4.0, 5.0, 0.0, 8.0]


/rh/tactile

This topic is published by the driver at 100 Hz with data from tactile sensors.

Example topic message when using PST fingertip sensors :

header:
  seq: 126618
  stamp:
    secs: 1528813967
    nsecs: 440903704
  frame_id: "rh_distal"
pressure: [405, 428, 422, 401, 384]
temperature: [1224, 1198, 1225, 1242, 1266]
Example topic message when using BioTac fingertip sensors :

tactiles:
-
pac0: 2048
pac1: 2054
pdc: 2533
tac: 2029
tdc: 2556
electrodes: [2622, 3155, 2525, 3062, 2992, 2511, 3083, 137, 2623, 2552, 2928, 3249, 2705, 3037, 3020, 2405, 3049, 948, 2458, 2592, 3276, 3237, 3244, 3119]
-
pac0: 0
pac1: 0
pdc: -9784
tac: 32518
tdc: 0
electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
-
pac0: 0
pac1: 0
pdc: -9784
tac: 32518
tdc: 0
electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
-
pac0: 0
pac1: 0
pdc: -9784
tac: 32518
tdc: 0
electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
-
pac0: 0
pac1: 0
pdc: -9784
tac: 32518
tdc: 0
electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


/rh_trajectory_controller/command

This topic can be published to and is the set position for the trajectory controller. It comprises an array of all the joints set positions and is used for commanding the robot.

For example the rqt joint sliders publish to it.

Example topic message :

joint_names: [rh_FFJ1, rh_FFJ2, rh_FFJ3, rh_FFJ4, rh_MFJ1, rh_MFJ2, rh_MFJ3, rh_MFJ4, rh_RFJ1,
rh_RFJ2, rh_RFJ3, rh_RFJ4, rh_LFJ1, rh_LFJ2, rh_LFJ3, rh_LFJ4, rh_LFJ5, rh_THJ1,
rh_THJ2, rh_THJ3, rh_THJ4, rh_THJ5, rh_WRJ1, rh_WRJ2]
points:
-
positions: [0.24434609527920614, 0.8203047484373349, 0.8552113334772214, -0.17453292519943295, 1.0297442586766545, 1.4311699866353502, 1.413716694115407, 0.007182575752410699, 0.9773843811168246, 1.5707963267948966, 1.2566370614359172, -0.12217304763960307, 0.4014257279586958, 1.2566370614359172, 1.5184364492350666, 0.017453292519943295, 0.13962634015954636, 0.12217304763960307, 0.6632251157578453, 0.17453292519943295, 1.117010721276371, -0.7504915783575618, -0.03490658503988659, 0.0]
velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
accelerations: []
effort: []
time_from_start:
secs: 0
nsecs: 5000000


/rh_trajectory_controller/state

This topic is read-only and update at 50 Hz from the trajectory controller with the positions and velocities of all 24 joints.

Example topic message :

positions: [0.0029928404547430176, 0.0007821521859359137, 0.004102784627362688, -0.001230489872427576, 0.002876479952986344, 0.0006426181816490129, 0.006354919224207833, 0.00213663812281073, 0.003279618063753098, 0.0020929781564538175, 0.0063066586043154516, 0.0038023568140372888, -0.002289758750686488, -1.1040675065743244e-05, 0.008137524637908733, -2.1288137004304986e-05, 0.0009348013388894572, -0.003295237358051928, 0.039981480504079236, -0.0035961821430152696, 0.0032603043080507987, 2.9988784142176428e-05, -0.00029934074598525484, -8.999634459527783e-05]
velocities: [-0.0008510441551395189, -0.0008510441551395189, 0.00016883698712266695, 0.00034715798956923955, -0.00017869100331692196, -0.00017869100331692196, -0.001275520583476054, -0.0004885423191519772, 0.00012555078906251334, 0.00012555078906251334, 0.0028653614401722843, -0.0008023399951605057, 0.0011760287859774613, 0.0011760287859774613, -0.0005423468659163991, -0.00017066612487367117, 0.0003102610817406156, -0.001127052578802167, -0.001465708865391472, -0.00028520412005307133, -0.00029795158858164227, 0.0002596403670543647, -5.819600689424957e-05, -0.0002980347643777659]


/sh_rh_*_position_controller/command

These topics can be published to and are the set position of each joint in radians. The topics are subscribed to by the driver (/sr_hand_robot node). This topic is used to communicate the set position with the rqt Joint Sliders plugin, when using position control. The Hand can be set to position control using the Change Controllers rqt plugin.

Example of running rostopic info /sh_rh_ffj0_position_controller/command :

Type: std_msgs/Float64
Publishers:

/rqt_gui_py_node_23644 (http://shadow-bravo:38385/)
Subscribers:

/sr_hand_robot (http://shadow-bravo:45091/)

/rostopic_15687_1526406188893 (http://shadow-bravo:36637/)

/record (http://shadow-bravo:35575/)
Example topic message :

data: 0.628318530718


/sh_rh_*_position_controller/state

These topics are published at 87 Hz by the driver (/sr_hand_robot node). They contain messages of type control_msgs/JointControllerState, which contain the parameters used for the each joints position controller.

Example topic message :

set_point: 1.1113358647
process_value: 1.11095072243
process_value_dot: 0.000426142920695
error: 0.0
time_step: 0.001
command: 0.0
p: -3800.0
i: 0.0d: 0.0
i_clamp: 0.0
antiwindup: False


/sh_rh_*_position_controller/max_force_factor

The "/sh_rh_*_position_controller/max_force_factor" topic can be published to and scales down the maximum output command of the joints position controller. The output command is interpreted by the driver (/sr_hand_robot node) as PWM if the driver is in PWM mode, or as tendon force if it are in Torque mode.
The maximum force is controlled by the parameter "max_force" that is specified in this yaml file : https://github.com/shadow-robot/sr-config/blob/kinetic-devel/sr_ethercat_hand_config/controls/host/rh/sr_edc_joint_position_controllers_PWM.yaml#L9
"max_force_factor" has a value between [0.0, 1.0] and controls the percentage of the `max_force` that will be effectively considered.
This parameter doesn't exist in the grasp controller.



/sh_rh_*_position_controller/pid/parameter_descriptions
/sh_rh_*_position_controller/pid/parameter_updates

These topics are read-only and contain parameters used for tuning the position controllers. They should not be published to directly and are accessed through rqt_reconfigure :




/tf
/tf_static

A "tf" is a transform in ROS. These topics store information on the active tfs in the ROS environment and holds their position and orientation in relation their parents. Static tfs are fixed and the dynamic tfs update at 100 Hz.
They can be published to, as well and read from.

For further information on ROS tfs see the ROS wiki : http://wiki.ros.org/tf

Example topic message :

transforms:
-
header:
     seq: 0
     stamp:
       secs: 1526995980
       nsecs: 100275357
     frame_id: "rh_ffmiddle"
    child_frame_id: "rh_ffdistal"
    transform:
     translation:
       x: 0.0
       y: 0.0
       z: 0.025
     rotation:
       x: 0.641034853577
       y: 0.0
       z: 0.0
       w: 0.767511769617
-
    header:
     seq: 0
     stamp:
       secs: 1526995980
       nsecs: 100275357
     frame_id: "rh_ffproximal"
    child_frame_id: "rh_ffmiddle"
    transform:
     translation:
       x: 0.0
       y: 0.0
       z: 0.045
     rotation:
       x: 0.759399719795
       y: 0.0
       z: 0.0
       w: 0.650624365955


/mechanism_statistics
This topic is read-only and updates at 1 Hz with the attributes of each joint, for example :

position: 0.715602037549
velocity: 0.0
measured_effort: -11.088
commanded_effort: -10.799974692
is_calibrated: False
violated_limits: False
odometer: 0.0
min_position: 0.715218542352
max_position: 0.715985532746
max_abs_velocity: 0.0363159179688
max_abs_effort: 15.84


/ros_ethercat/motors_halted
This topic is deprecated - no longer used.
It is a read-only boolean value, updated at 1 Hz, which indicates if the motors have been halted. Generally the value of this is true : http://wiki.ros.org/ethercat_hardware


/rosout
/rosout_agg
This is the ROS console log reporting mechanism : http://wiki.ros.org/rosout
The ROS core node, rosout subscribes to the standard /rosout topic, records these messages in a textual log file, and rebroadcasts the messages on /rosout_agg

Moveit! Topics
In Position control the Moveit topics are used for trajectory planning.
It should not be necessary to interface with these topics, which are described in their documentation here : https://moveit.ros.org/documentation/

These topics provide information about positions, velocities and accelerations of joints whilst executing a trajectory from the current pose to the goal pose.
/rh_trajectory_controller/follow_joint_trajectory/cancel
Used to stop a currently executing trajectory.
/rh_trajectory_controller/follow_joint_trajectory/feedback
/rh_trajectory_controller/follow_joint_trajectory/goal
/rh_trajectory_controller/follow_joint_trajectory/result
/rh_trajectory_controller/follow_joint_trajectory/status

/attached_collision_object
/collision_object
These are used for object collision avoidance if it is active.
/execute_trajectory/cancel
/execute_trajectory/feedback
/execute_trajectory/goal
/execute_trajectory/result
/execute_trajectory/status
Live information regarding the current trajectory execution.
/move_group/cancel
/move_group/display_contacts
/move_group/display_planned_path
/move_group/feedback
/move_group/goal
/move_group/monitored_planning_scene
/move_group/ompl/parameter_descriptions
/move_group/ompl/parameter_updates
/move_group/plan_execution/parameter_descriptions
/move_group/plan_execution/parameter_updates
/move_group/planning_scene_monitor/parameter_descriptions
/move_group/planning_scene_monitor/parameter_updates
/move_group/result
/move_group/sense_for_plan/parameter_descriptions
/move_group/sense_for_plan/parameter_updates
/move_group/status
/move_group/trajectory_execution/parameter_descriptions
/move_group/trajectory_execution/parameter_updates
Information from the move_group node : https://moveit.ros.org/documentation/concepts/
/pickup/cancel
/pickup/feedback
/pickup/goal
/pickup/result
/pickup/status
/place/cancel
/place/feedback
/place/goal
/place/result
/place/status
/planning_scene
/planning_scene_world
/recognized_object_array
/trajectory_execution_event
/filtered

RViz Topics
These topics are used to interface with RViz. Documentation for this can be found here : http://wiki.ros.org/rviz#User_Documentation
/rviz_*/motionplanning_planning_scene_monitor/parameter_descriptions
/rviz_*/motionplanning_planning_scene_monitor/parameter_updates
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full



### Using rosservice
To reset individual motors, E.G. FFJ3:
  ```bash
	$ rosservice call /realtime_loop/reset_motor_FFJ3
  ```
To change control modes, E.G. teach mode:
  ```bash
	$ rosservice call /realtime_loop/xxxxxx
  ```

## Writing controllers
Rather than use the ROS topics to access sensor data, you will need to write a plugin for the PR2 Controller Manager. This will give you access to the sensor data at the full 1kHz rate, and allow you to create your own control algorithms for the hand. Please see this page for more information about the PR2 Controller Manager:
	<http://ros.org/wiki/pr2_controller_manager>

The Controller Manager is the node that talks to the hardware via EtherCAT and provides a facility for hosting plugins. The position controllers you have already used are examples of this. Note that the Controller Manager can host any number of running controllers but one should be loaded at a time for a given joint so they don't fight for control.

## Deeper settings
### Editing PID settings
The motor controller PID settings are stored in a YAML files. You can find the files in the next folder:
  ```bash
	$ roscd sr_ethercat_hand_config/controls/
  ```
###  Changing motor data update rates
Each motor can return two sensor readings every 2ms. The first is always the measured torque. The second is requested by the host. This allows the host to decide on the sensor update rate of each sensor. Currently, the rates cannot be adjusted at run-time, and are specified in a file which you can edit. To edit the file:
  ```bash
  $ roscd sr_robot_lib/config
  $ gedit motor_data_polling.yaml
  ```

The complete list of motor sensors appears in the file, along with a number
```eval_rst
=======     ===========================
Number      Meaning
=======     ===========================
-2          Read once when the driver is launched
-1          Read as fast as possible
 0          Do not use zero
>0          Read period in seconds
=======     ===========================
```

Sensors set to -1 will be read in turn, unless it's time to read another sensor. Usually 5 sensors are set to -1, meaning that they are sampled at 100Hz.
