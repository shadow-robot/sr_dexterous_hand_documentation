Robot descriptions (URDF)
==========================

We currently have modular xacro files for our robots including hands and arms setups, allowing the robots to start in various configurations. They can be found in our `sr_description <https://github.com/shadow-robot/sr_common/tree/noetic-devel/sr_description>`_ and `sr_interface <https://github.com/shadow-robot/sr_common/tree/noetic-devel/sr_interface>`_ packages.

Shadow Hands
--------------

Unimanual
~~~~~~~~~~

The main xacro file to use is `sr_hand.urdf.xacro <https://github.com/shadow-robot/sr_common/tree/noetic-devel/sr_description/robots/sr_hand.urdf.xacro>`_ when you are using only one of our hands.

The following arguments are available:

* ``side`` - defines the side of the hand. Allowed options: ``right``/``left``
* ``hand_type`` - defines the type of the hand. Allowed options: ``hand_e``/``hand_g``/``hand_c``
* ``hand_version`` - defines version for particular type of hand.
* ``fingers`` - defines which fingers does the hand have, can be ``all`` or a string in a format of ``th,ff,mf,rf,lf`` 

Current allowed configurations are the following:

+--------------+----------------+---------------------+---------------------------+--------------------------+
|              | Dexterous Hand | Dexterous Hand Lite | Dexterous Hand Extra Lite | Muscle_hand (deprecated) |
+==============+================+=====================+===========================+==========================+
| hand_type    |     hand_e     |        hand_g       |           hand_g          |          hand_c          |
+--------------+----------------+---------------------+---------------------------+--------------------------+
| hand_version |   E3M5, E2M3   |         G1M5        |            G1M5           |           C6M2           |
+--------------+----------------+---------------------+---------------------------+--------------------------+
| fingers      |       all      |         all         |            all            |            all           |
|              +----------------+---------------------+---------------------------+--------------------------+
|              | th,ff,mf,rf,lf |     th,ff,mf,rf     |          th,ff,mf         |      th,ff,mf,rf,lf      |
+--------------+----------------+---------------------+---------------------------+--------------------------+

There are also arguments that define where and which sensors are located on the hand. It allows placement of sensors on tip, mid and proximal parts of the fingers as well as the palm. Argument names: ``tip_sensors``, ``mid_sensors``, ``prox_sensors``, ``palm_sensor``. Currently, only sensors at the fingertips are available. There are three fingertip sensor types: ``pst``/``bt_sp``/``bt_2p``.

+-------------+-----+------------------+
|             | PST | Syntouch Biotacs |
|             |     +---------+--------+
|             |     |    2p   |   sp   |
+=============+=====+=========+========+
| tip_sensors | pst |  bt_2p  |  bt_sp |
+-------------+-----+---------+--------+

Bimanual
~~~~~~~~~

If you have a setup with two robot hands, this is the xacro to use: `sr_hand_bimanual.urdf.xacro <https://github.com/shadow-robot/sr_common/tree/noetic-devel/sr_description/robots/sr_hand_bimanual.urdf.xacro>`_

The following arguments are available (similar to the hand-only scenario but with the side prefix to specify every configuration):

* ``right_hand_type``
* ``right_hand_version``
* ``right_fingers``
* ``right_tip_sensors``
* ``right_mid_sensors``
* ``right_prox_sensors``
* ``right_palm_sensor``
* ``left_hand_type``
* ``left_hand_version``
* ``left_fingers``
* ``left_tip_sensors``
* ``left_mid_sensors``
* ``left_prox_sensors``
* ``left_palm_sensor``

Shadow Hands mounted on UR arms
--------------------------------
The main xacros for Universal Robot Arms and Shadow hand systems are: 

Unimanual
~~~~~~~~~~

* `srhand_ur.urdf.xacro <https://github.com/shadow-robot/sr_interface/tree/noetic-devel/sr_multi_description/urdf/srhand_ur.urdf.xacro>`_ 

Additional parameters:

* ``robot_model`` - defines which robot model is used. Allowed options: ``ur10``/``ur10e``/``ur5``/``ur5e``
* ``initial_z`` - defines how high above the ground the robot is spawned

Bimanual
~~~~~~~~~

* Bimanual arms: `bimanual_ur.urdf.xacro <https://github.com/shadow-robot/sr_interface/tree/noetic-devel/sr_multi_description/urdf/bimanual_ur.urdf.xacro>`_
* Bimanual arms and hands; `bimanual_srhand_ur.urdf.xacro <https://github.com/shadow-robot/sr_interface/tree/noetic-devel/sr_multi_description/urdf/bimanual_srhand_ur.urdf.xacro>`_
  
Additional parameters:

* ``robot_model`` - defines which robot model is used. Allowed options: ``ur10``/``ur10e``/``ur5``/``ur5e``
* ``arm_1_z`` - defines how high above the ground the right robot arm is spawned
* ``arm_2_z`` - defines how high above the ground the left robot arm is spawned
* ``arm_x_separation`` - x separation of the left arm with respect to the right arm
* ``arm_y_separation`` - y separation of the left arm with respect to the right arm


Usage
---------------------------------

For usage example, refer to the xacro files themselves or the `unimanual <https://github.com/shadow-robot/sr_interface/tree/noetic-devel/sr_robot_launch/launch/load_robot_description.launch>`_ and `bimanual <https://github.com/shadow-robot/sr_interface/tree/noetic-devel/sr_robot_launch/launch/load_robot_description_bimanual.launch>`_ launchfiles that use them.
When used with Shadow Hands all the hand parameters are automatically set for you with the autodetection. However, if you are running in simulation or just want to omit the autodetection and set them manually, you can pass the args directly to the launchfile or xacro command. The following are examples on how to use them.

* Launch file:

  .. code-block::

     roslaunch sr_robot_launch srhand.launch side:=right hand_type:=hand_g hand_version:=G1M5 fingers:=th,ff,mf,rf,lf tip_sensors:=ff=bt_2p,lf=bt_sp,mf=pst,rf=pst,th=bt_sp mid_sensors:=none prox_sensors:=none palm_sensor:=none sim:=true

* Xacro command:

  .. code-block::

     xacro <xacro file> side:=right hand_type:=hand_g hand_version:=G1M5 fingers:=th,ff,mf,rf,lf tip_sensors:=ff=bt_2p,lf=bt_sp,mf=pst,rf=pst,th=bt_sp mid_sensors:=none prox_sensors:=none palm_sensor:=none

As far as SRDF’s are concerned, all necessary ones are autogenerated from ``robot_description`` ros parameters spawned to the parameter server.

Autodetection parameters
--------------------------

For each of the hands, there is a ``general_info.yaml`` file that contains information about the hand and will be used to pass correct arguments to the launchfiles, and further to the xacros. When hand is being autodetected, the script will look into that file, extract all necessary arguments and provide them to the launchfile as a command suffix. All of the "general info" files can be found in `sr_hand_config <https://github.com/shadow-robot/sr_hand_config>`_ repository, inside hand serial folder corresponding to each particular hand.
