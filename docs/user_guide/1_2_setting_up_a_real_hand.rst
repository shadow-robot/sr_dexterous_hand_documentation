Setting up a real hand
=======================

Unimanual system
------------------

What's in the box?
------------------

=============================   ==================================================================
Item                            Description
=============================   ==================================================================
Shadow Hand E2M3 or E2PT        Hand Unit
NUC control machine             i7 NUC minicomputer for running hand's driver
NUC power supply                Power supply for the NUC computer
USB->Ethernet adapter x3        Adapters for connections between NUC, hand and client PC
PSU for Hand                    48v for motor hand
Kettle Leads                    To connect power supplies to mains
Power Cable                     4-pin Large Lemo connector, already fitted to the hand
EtherCAT Extension Cable        50cm EtherCAT extension lead, already fitted to the Hand
Ethernet Cable x2               1m ethernet cables to connect computers and the hand
Calibration Jigs                Bag containing calibration jigs for all joints
Toolbox                         Contains hex drivers to perform required maintenance
User Manual                     This document
Client PC (optional)            3XS laptop as control unit. Power supply and mouse included
64GB USB pendrive               Clonezilla backup copies of the NUC and (optionally) the client PC
Hand programmer                 Hand firmware programmer
Mounting plate with screws      Mounting plate allowing the hand to be assembled on a UR10 robot
=============================   ==================================================================

Connecting Cables
------------------
There are two ways to connect the EtherCAT and power cables to the hand.

External connections
^^^^^^^^^^^^^^^^^^^^^
If your hand already has cables fitted, then you can simply connect the EtherCAT and power connectors immediately.
.. figure:: ../img/connecting_cables_external.png
    :width: 200px
    :align: center
    :alt: Connecting cables

    Connecting cables

**EtherCAT**: Connect the Ethernet cable to the hand's Ethernet socket, and connect the other end to the USB->Ethernet adapter with a label `HAND`. Then, connect the USB end of the adapter to any of the USB ports in the NUC. Next, connect USB->Ethernet adapter with a label `NUC-CONTROL` to another USB port on the NUC and adapter with a label `SERVER` to any of the ports in your client PC (provided by Shadow or a custom one). Finally, connect the two adaptors together with an Ethernet cable.
You have been supplied with medium length Ethernet leads, but if you require a different length, you can simply use a standard commercial Ethernet Cat 5 cable, available from most computer parts suppliers. If you require internet connection in the laptop, connect an ethernet cable providing external internet connection to the back of the laptop, to an ethernet port labelled `INTERNET`.

![Connections_diagram](../img/hand_connections_diagram.png)

**Power**: Connect the external power supply to the hand using the metal Lemo connector, making sure to line up the red dots. If you require a longer or shorter cable, please contact the Shadow Robot Company.

Internal connections
^^^^^^^^^^^^^^^^^^^^
If you are connecting the hand to a robot with internal cabling, then you may wish to use the internal connectors.
Turn the hand over, and use the orange and green hex drivers to remove the connector cover. Connect the two cables to their relevant sockets. Now affix the hand to your robot arm. The rest of the connection steps remain the same as in the section above.
![Connecting cables](../img/connecting_cables_internal.png)

Mounting the hand
-----------------
Shadow Robot can supply an elbow adaptor plate to adapt the Hand to most other robot arms. However, if you wish to make your own fitting for the Hand:
![Mounting the hand](../img/mounting_hand.png)

The Hand's elbow plate contains eight screw holes which accept M6 bolts to a depth of 12mm. The holes are spaced equally from the centre on a circle with diameter 100mm. The overall diameter of the elbow plate is 135mm

To mount the hand properly and align with our xacros you need to rotate it as shown in the picture below:
![Aligning the hand](../img/arm_hand.png)

The hand's palm points in the direction of the TCP point of the arm. 

Powering up
-----------
You can power up the hand and PC in any order. You do not have to power up one before the other. When power is applied to the hand, the fans will be heard immediately.

Lights
^^^^^^

On power up, the lights will be in the following state:

=======================   =============       ================    =================================
Item                      Color               Activity            Meaning
=======================   =============       ================    =================================
Power LEDs                White               On                  Power good
EC Link Active            Green               On                  EtherCAT link established
EC Link Error             Red                 Off                 No EtherCAT link error
Run                       Green               Off                 Hand is in Init state
Application Layer Error   Red                 On (during boot)    Verifying ET1200 EEPROM
Application Layer Error   Red                 Then off            No EtherCAT packet error
ET1200 chip select        Yellow              On                  PIC32 communicating with ET1200
=======================   =============       ================    =================================

Lights will also appear inside the base, indicating 5v, 6v and 24v (or 28v) supplies. These can only be seen by removing the covers.

Jiggling
^^^^^^^^

This applies to the motor hand only. On reset, all of the strain gauges (torque sensors) in the
motors need to be zeroed. This happens automatically. The motors are driven back and forth
to try to relieve any tension on the tendons. Then both gauges are zeroed. You will therefore
see all joints of the hand move slightly on power up or reset.

Installing the software
-----------------------
By default, we will provide machines that already have all the software set up for you. However, even though each delivery will consist of a NUC machine for Hand's driver, the client PC is optional. In case you want to set up a custom machine as a client, please follow the instructions below.

#### On a new PC using the one-liner
We have created a one-liner that is able to install Docker, download the docker image and create a new container for you. It will also create desktop icons, one to start the container, one to launch the hand driver on the control box and one to save the log files locally. To use it, you first need to have a PC with Ubuntu installed on it (preferably version 16.04), then follow these steps:

* **Get ROS Upload login credentials**

  If you want to upload technical logged data (ROS logs, backtraces, crash dumps etc.) to our server and notify the Shadow's software team to investigate your bug, then you need to enable logs uploading in the one-liner. In order to use this option you need to obtain a unique upload key by emailing sysadmin@shadowrobot.com. When you receive the key you can use it when running the one-liner installation tool. To enable the logs uploading you need to add the command line option ```use_aws=true``` to the one-liner.
  After executing the one-liner, it will prompt you to enter your upload key and press enter to continue. Please copy and paste your key from the email you received from Shadow Robot.

* **Run the one-liner**:

  The one-liner will install Docker, pull the image from Docker Hub, and create and run a container with the parameters specified. In order to use it, run the following command:

  ROS Kinetic (Recommended):

.. prompt:: bash $

   bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure sudo_password ethercat_interface=<ethercat_interface> config_branch=<config_branch> product=hand_e reinstall=true hand_serial=<hand_serial> internet_interface_name=<internet_interface_name> dhcp_interface_name=<dhcp_interface_name> dhcp_server_mac=<dhcp_server_mac> dhcp_client_mac=<dhcp_client_mac> upgrade_check=true launch_hand=true

  where `<ethercat_interface>`, `<config_branch>`, `<hand_serial>`, `<internet_interface_name>`, `<dhcp_interface_name>`, `<dhcp_server_mac>` and `<dhcp_client_mac>` are values that will be provided by Shadow.

  An example of the script with ROS logs upload enabled:

.. prompt:: bash $

   bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure sudo_password,customer_key ethercat_interface=enx000ec6511588 config_branch=shadowrobot_200117 product=hand_e reinstall=true use_aws=true hand_serial=2378 internet_interface_name=enp8s0f1 dhcp_interface_name=enx000ec653b3bc dhcp_server_mac="00:0e:c6:53:b3:bc" dhcp_client_mac="00:0e:c6:53:b4:35" upgrade_check=true launch_hand=true

  In another example, if you do not have an Nvidia graphics card, you can add nvidia_docker=false to use nvidia-docker (`true` is our default), i.e.:

.. prompt:: bash $

   bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure sudo_password,customer_key ethercat_interface=enx000ec6511588 config_branch=shadowrobot_200117 product=hand_e reinstall=true use_aws=true hand_serial=2378 internet_interface_name=enp8s0f1 dhcp_interface_name=enx000ec653b3bc dhcp_server_mac="00:0e:c6:53:b3:bc" dhcp_client_mac="00:0e:c6:53:b4:35" upgrade_check=true launch_hand=true nvidia_docker=false

  You can also add `reinstall=true` in case you want to reinstall the docker image and container. When it finishes it will show if it was successful or not
  and will create desktop icons on your desktop that you can double-click to launch the hand container, save the log files from the active containers to your desktop and perform various actions on the hand (open, close and demo).
  The icons look like this:

  ![Icons](../img/icons.png)

  - Launch Shadow Hand - launches the hand
  - Shadow ROS Logs Saver - used to save the hand logs and upload them to AWS
  - Shadow NUC RQT - opens RQT window running within the NUC machine, allows access to ROS plugins

  Within the `Shadow Demos` folder you will find following icons (use only when driver is running):

  ![Shadow Demos](../img/shadow_demos.png)

  - Close Right Hand - moves hand into pack position
  - Demo Right Hand - starts a program running several hand demos
  - Open Right Hand - moves hand into fully open position

  Within the `Shadow Advanced Launchers` folder you will find following icons:

  ![Shadow Advanced Launchers](../img/shadow_advanced_launchers.png)

  - Launch Server Container - starts docker container on the server machine only
  - Launch Server ROSCORE - only starts roscore on the server side
  - Launch NUC Container and Hardware Control Loop - starts the hand driver only, on the NUC side
  - Launch Server GUI - Start GUI on the server side allowing user to control movements of the hand

  The above four icons run in succession are the equivalent of using the `Launch Shadow Hand` icon.

  - Launch Local Shadow Hand - icon to start the hand when it is plugged directly in to the server machine
  - Launch NUC container - start docker container on the NUC without starting the driver

Using a PC that Shadow provided
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
In this case, the previous steps would already have been performed by the Shadow team and the only thing to do is start the docker container by double-clicking the desktop icon.

Saving log files and uploading data to our server
--------------------------------------------------
When running the one-liner, along with the icon that starts the Dexterous Hand, you will also notice a second icon named 'Save Logs' that is used to retrieve and copy all the available logs files from the active containers locally on your Desktop. This icon will create a folder that matches the active container's name and the next level will include the date and timestamp it was executed. When it starts, it will prompt you if you want to continue, as by pressing yes it will close all active containers. After pressing "yes", you will have to enter a description of the logging event and it will start copying the bag files, logs and configuration files from the container and then exit. Otherwise, the window will close and no further action will happen. If you provided an upload key with the one-liner installation then the script will also upload your LOGS in compressed format to our server and notify Shadow's software team about the upload. This will allow the team to fully investigate your issue and provide support where needed.

Starting the driver
-------------------

* **Shadow Hand Driver**
  Launch the driver for the Shadow Hand using the desktop icon 'Launch Hand' or, if you want to launch the hand locally, plug in the hand ethernet adapter to the laptop and use the Advanced Launch Icon - `Launch Local Shadow Hand`.

* **Lights in the hand**:
  When the ROS driver is running you should see the following lights on the Palm:

  ========================   =============       ================    =================================
  Light                      Colour              Activity            Meaning
  ========================   =============       ================    =================================
  Run                        Green               On                  Hand is in Operational state
  CAN1/2 Transmit            Blue                V.fast flicker      Demand values are being sent to the motors
  CAN1/2 Receive             Blue                V.fast flicker      Motors are sending sensor data
  Joint sensor chip select   Yellow              On                  Sensors being sampled
  ========================   =============       ================    =================================

  After killing the driver, the lights will be in a new state:

  ========================   =============       ================    =================================
  Light                      Colour              Activity            Meaning
  ========================   =============       ================    =================================
  Run                        Green               Blinking            Hand is in Pre-Operational state
  CAN1/2 Transmit            Blue                Off                 No messages transmitted on CAN 1/2
  CAN1/2 Receive             Blue                Off                 No messages received on CAN 1/2
  Joint sensor chip select   Yellow              Off                 Sensors not being sampled
  ========================   =============       ================    =================================

Bimanual system
------------------