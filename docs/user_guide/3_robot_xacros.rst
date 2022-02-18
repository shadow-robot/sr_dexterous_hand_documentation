Robot xacros
==========================

We currently have modular xacro files for our hand, and hand and arm setups, allowing the robots to start in various configurations.

Unimanual
--------------------
There are two main xacro files that can be used - `sr_hand.urdf.xacro <https://github.com/shadow-robot/sr_common/blob/F_new_xacros_ready/sr_description/robots/sr_hand.urdf.xacro>`_ (hand only) and `srhand_ur.urdf.xacro <https://github.com/shadow-robot/sr_interface/blob/F_new_xacros_ready/sr_multi_description/urdf/srhand_ur.urdf.xacro>`_ (hand and arm).

For hand only xacro following arguments are available:

* ``side`` - defines the side of the hand. Allowed options: ``right``/``left``
* ``hand_type`` - defines the type of the hand. Allowed optins: ``hand_e``/``hand_g``/``hand_c``
* ``hand_version`` - defines version for particular type of hand. Allowed options: ``E3M5``/``E2M3`` (for hand_e) ``G1M5`` (for hand_g) ``C6M2`` (for hand_c)
* ``fingers`` - defines which fingers does the hand have, can be ``all`` or a string in a format of ``th,ff,mf,rf,lf`` 

There are also arguments that define where and which sensors are located on the hand. It allows placement of sensors on tip, mid and proximal parts of the fingers as well as the palm. Argument names: ``tip_sensors``, ``mid_sensors``, ``prox_sensors``, ``palm_sensor``. Allowed sensor types: ``pst``/``bt_sp``/``bt_2p``.

Additionally, for arm and hand xacro:

* ``robot_model`` - defines which robot model is used. Allowed options: ``ur10``/``ur10e``/``ur5``/``ur5e``
* ``initial_z`` - defines how high above the ground the robot is spawned

For arm only setups, separate xacros are available `here <https://github.com/shadow-robot/sr_interface/tree/noetic-devel/sr_multi_moveit/sr_box_ur10_moveit_config/config>`_.

Bimanual
----------------------------------------

There are additional xacros for bimanual setups: `sr_hand_bimanual.urdf.xacro <https://github.com/shadow-robot/sr_common/blob/F_new_xacros_ready/sr_description/robots/sr_hand_bimanual.urdf.xacro>`_ (hands only), `bimanual_ur.urdf.xacro <https://github.com/shadow-robot/sr_interface/blob/F_new_xacros_ready/sr_multi_description/urdf/bimanual_ur.urdf.xacro>`_ (arms only) and `bimanual_srhand_ur.urdf.xacro <https://github.com/shadow-robot/sr_interface/blob/F_new_xacros_ready/sr_multi_description/urdf/bimanual_srhand_ur.urdf.xacro>`_ (arms and hands). For these, the side argument is not being provided and each of the hand arguments above are prefixed with  ``right`` and ``left`` strings. There are also separation arguments available to define the distance between the robots.

Usage
---------------------------------

For usage example, refer to the xacro files themselves or the `unimanual <https://github.com/shadow-robot/sr_interface/blob/F_new_xacros_ready/sr_robot_launch/launch/load_robot_description.launch>`_ and `bimanual <https://github.com/shadow-robot/sr_interface/blob/F_new_xacros_ready/sr_robot_launch/launch/load_robot_description_bimanual.launch>`_ launchfiles that use them.
When used with Shadow Hands all the hand parameters are automatically set for you with the autodetection. However, if you are running in simulation or just want to omit the autodetection and set them manually, you can pass the args directly to the launchfile or xacro command. Example usage:

  .. prompt:: bash $

     roslaunch sr_robot_launch srhand.launch side:=right hand_type:=hand_g hand_version:=G1M5 fingers:=th,ff,mf,rf,lf tip_sensors:=ff=bt_2p,lf=bt_sp,mf=pst,rf=pst,th=bt_sp mid_sensors:=none prox_sensors:=none palm_sensor:=none sim:=true

or

  .. prompt:: bash $

     xacro <xacro file> side:=right hand_type:=hand_g hand_version:=G1M5 fingers:=th,ff,mf,rf,lf tip_sensors:=ff=bt_2p,lf=bt_sp,mf=pst,rf=pst,th=bt_sp mid_sensors:=none prox_sensors:=none palm_sensor:=none

As far as SRDF’s are concerned, all necessary ones are autogenerate from ``robot_description`` ros parameters spawned to the parameter server.
