Application Programming Interfaces (APIs)
=========================================

There are two different APIs for our systems:

* Direct ROS Interface using `rospy <http://wiki.ros.org/rospy>`_ and `roscpp <http://wiki.ros.org/roscpp>`_
* Shadow Robot Interfaces which provide high level interfaces to easily interact with and control the different robots supported by Shadow Robot.

Direct ROS Interfacing
----------------------

Direct ROS Interface using `rospy <http://wiki.ros.org/rospy>`_ and `roscpp <http://wiki.ros.org/roscpp>`_

Shadow Robot Interfacing
--------------------------

The robot commander encapsulates functionality provided by different ROS packages, especially the moveit_commander, providing access via a simplified interface.

There are three classes available:

* `SrRobotCommander <https://github.com/shadow-robot/sr_interface/blob/noetic-devel/sr_robot_commander/src/sr_robot_commander/sr_robot_commander.py>`_: base class.
* `SrHandCommander <https://github.com/shadow-robot/sr_interface/blob/noetic-devel/sr_robot_commander/src/sr_robot_commander/sr_hand_commander.py>`_: hand management class.
* `SrArmCommander <https://github.com/shadow-robot/sr_interface/blob/noetic-devel/sr_robot_commander/src/sr_robot_commander/sr_arm_commander.py>`_: arm management class.

SrRobotCommander
^^^^^^^^^^^^^^^^^

Overview
~~~~~~~~~

The main purpose of the robot commander is to provide a base class to the
hand commander. The RobotCommander should not be used directly unless necessary.
Use the ``SrHandCommander`` instead.

Examples of usage can be found `here <https://github.com/shadow-robot/sr_interface/tree/noetic-devel/sr_example/scripts/sr_example>`_.

In the following sections, you can find decriptions of the most relevant functions of the hand commander.

Basic terminology
~~~~~~~~~

A robot is described using an `srdf <http://wiki.ros.org/srdf>`_ file which contains the semantic description that is not available in the `urdf <http://wiki.ros.org/urdf>`__. It describes a robot as a collection of **groups** that are representations of different sets of joints that are useful for planning. Each group can have its **end-effector** and **group states** specified. Group states are a specific set of joint values predefined for a group with a given name, for example *close_hand* or *open_hand*.

As the robot commander is a high level wrapper of the `moveit_commander <http://wiki.ros.org/moveit_commander>`_, its constructor takes the name of one of the robot groups for which the planning will be performed.

Setup
~~~~~~~~~

Import the hand commander along with basic rospy libraries:

.. code-block:: python

    import rospy
    from sr_robot_commander.sr_hand_commander import SrHandCommander

The constructor for the ``SrHandCommander`` takes a
name parameter that should match the group name of the robot to be used.

As well as creating an instance of the ``SrHandCommander`` class, we must also initialise our ros node:

.. code-block:: python

    rospy.init_node("sr_hand_commander_example", anonymous=True)
    hand_commander = SrHandCommander("right_hand")

Getting basic information
~~~~~~~~~

We can get the name of the robot, group or planning reference frame:

.. code-block:: python

    print("Robot name: ", hand_commander.get_robot_name())
    print("Group name: ", hand_commander.get_group_name())
    print("Planning frame: ", hand_commander.get_planning_frame())

Get the list of names of the predefined group states from the srdf and warehouse for the current group:

.. code-block:: python

   # Refresh them first if they have recently changed
   hand_commander.refresh_named_targets()

   print("Named targets: ", hand_commander.get_named_targets())

Get the joints position and velocity:

.. code-block:: python

    joints_position = hand_commander.get_joints_position()
    joints_velocity = hand_commander.get_joints_velocity()

    print("Hand joint positions\n" + str(joints_position) + "\n")
    print("Hand joint velocities\n" + str(joints_velocity) + "\n")

Get the current joint state of the group being used:

.. code-block:: python

   current_state = hand_commander.get_current_state()

   # To get the current state while enforcing that each joint is within its limits
   current_state = hand_commander.get_current_state_bounded()

Setting functions
~~~~~~~~~~~~~~~~~~
You can change the reference frame to get pose information:

.. code-block:: python

   hand_commander.set_pose_reference_frame("palm")

You can also activate or deactivate the teach mode for the robot:

.. code-block:: python

   # Activation: stops the trajectory controllers for the robot, and sets it to teach mode.
   hand_commander.set_teach_mode(True)

   # Deactivation: stops the teach mode and starts trajectory controllers for the robot.  
   # Currently, this method blocks for a few seconds when called on a hand, while the hand parameters are reloaded.
   hand_commander.set_teach_mode(False)

Plan/move to a joint-space goal
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Using the methods ``plan_to_joint_value_target``, ``move_to_joint_value_target`` or ``move_to_joint_value_target_unsafe``, a set of the joint values can be given for the specified group to create a plan and send it for execution.

Parameters:

-  *joint\_states* is a dictionary with joint name and value. It can
   contain joints' values of which need to be changed.
-  *wait* indicates if the method should wait for the movement to end or not
   (default value is True)
-  *angle\_degrees* should be set to true if the input angles are in
   degrees (default value is False)

*IMPORTANT:* Bear in mind that the names of the joints are different for
the right and left hand.

Example
++++++++

.. code-block:: python

    rospy.init_node("robot_commander_examples", anonymous=True)

    hand_commander = SrHandCommander(name="right_hand")
    joints_states = {'rh_FFJ1': 90, 'rh_FFJ2': 90, 'rh_FFJ3': 90, 'rh_FFJ4': 0.0,
                     'rh_MFJ1': 90, 'rh_MFJ2': 90, 'rh_MFJ3': 90, 'rh_MFJ4': 0.0,
                     'rh_RFJ1': 90, 'rh_RFJ2': 90, 'rh_RFJ3': 90, 'rh_RFJ4': 0.0,
                     'rh_LFJ1': 90, 'rh_LFJ2': 90, 'rh_LFJ3': 90, 'rh_LFJ4': 0.0, 'rh_LFJ5': 0.0,
                     'rh_THJ1': 40, 'rh_THJ2': 35, 'rh_THJ3': 0.0, 'rh_THJ4': 65, 'rh_THJ5': 15,
                     'rh_WRJ1': 0.0, 'rh_WRJ2': 0.0}
    hand_commander.move_to_joint_value_target(joints_states, wait=False, angle_degrees=True))

In this example, joint states for a hand are sent to the ``HandCommander``,
the method is prompted by the ``wait=False`` argument to not wait for the
movement to finish executing before moving on to the next command and
the ``angle_degrees=True`` argument tells the method that the input
angles are in degrees, so require a conversion to radians.

Plan/move to a predefined group state
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using the methods ``plan_to_named_target`` or ``move_to_named_target`` will allow to plan or move the group to a predefined pose. This pose can be defined in the srdf or saved as a group state in the moveit warehouse.

Parameters:

-  *name* is the unique identifier of the target pose
-  *wait* indicates if the method should wait for the movement to end or not
   (default value is True)

Example
++++++++

**pack** is a predefined pose defined in the SRDF file for the *right_hand* group:

.. code-block:: html

  <group_state group="right_hand" name="pack">
    <joint name="rh_THJ1" value="0.52"/>
    <joint name="rh_THJ2" value="0.61"/>
    <joint name="rh_THJ3" value="0.00"/>
    <joint name="rh_THJ4" value="1.20"/>
    <joint name="rh_THJ5" value="0.17"/>
    <joint name="rh_FFJ1" value="1.5707"/>
    <joint name="rh_FFJ2" value="1.5707"/>
    <joint name="rh_FFJ3" value="1.5707"/>
    <joint name="rh_FFJ4" value="0"/>
    <joint name="rh_MFJ1" value="1.5707"/>
    <joint name="rh_MFJ2" value="1.5707"/>
    <joint name="rh_MFJ3" value="1.5707"/>
    <joint name="rh_MFJ4" value="0"/>
    <joint name="rh_RFJ1" value="1.5707"/>
    <joint name="rh_RFJ2" value="1.5707"/>
    <joint name="rh_RFJ3" value="1.5707"/>
    <joint name="rh_RFJ4" value="0"/>
    <joint name="rh_LFJ1" value="1.5707"/>
    <joint name="rh_LFJ2" value="1.5707"/>
    <joint name="rh_LFJ3" value="1.5707"/>
    <joint name="rh_LFJ4" value="0"/>
    <joint name="rh_LFJ5" value="0"/>
    <joint name="rh_WRJ1" value="0"/>
    <joint name="rh_WRJ2" value="0"/>
  </group_state>

Here is how to move to it:

.. code-block:: python

    rospy.init_node("robot_commander_examples", anonymous=True)
    hand_commander = SrHandCommander(name="right_hand")

    # Only plan
    hand_commander.plan_to_named_target("pack")

    # Plan and execute
    hand_commander.move_to_named_target("pack")

Move through a trajectory of predefined group states
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using the method ``run_named_trajectory``, it is possible to specify a trajectory composed of a set of names of previously defined group states (either from SRDF or from warehouse), plan and move to follow it.

Parameters:

-  *trajectory* specifies a dictionary of waypoints with the following elements:
    -  name: the name of the waypoint
    -  interpolate_time: time to move from last waypoint
    -  pause_time: time to wait at this waypoint

Example
+++++++

.. code-block:: python

   trajectory = [
      {
          'name': 'open',
          'interpolate_time': 3.0
      },
      {
          'name': 'pack',
          'interpolate_time': 3.0,
          'pause_time': 2
      },
      {
          'name': 'open',
          'interpolate_time': 3.0
      },
      {
          'name': 'pack',
          'interpolate_time': 3.0
      }
   ]

   hand_commander.run_named_trajectory(trajectory)

   # If you want to send the trajectory to the controller without using the planner, you can use the unsafe method:
   hand_commander.run_named_trajectory_unsafe(trajectory)

Check if a plan is valid and execute it
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use the method ``check_plan_is_valid`` and ``execute`` to check if the current plan contains a valid trajectory and execute it. This only has meaning if called after a planning function has been attempted.

Example
++++++++

.. code-block:: python

  import rospy
  from sr_robot_commander.sr_hand_commander import SrHandCommander
  rospy.init_node("robot_commander_examples", anonymous=True)

  hand_commander = SrHandCommander()

  hand_commander.plan_to_named_target("open")
  if hand_commander.check_plan_is_valid():
      hand_commander.execute()

Stop the robot
~~~~~~~~~~~~~~~~~~
Use the method ``send_stop_trajectory_unsafe`` to send a trajectory with the current joint state to stop the robot at its current position.

Example
+++++++

.. code-block:: python

   hand_commander.send_stop_trajectory_unsafe()

SrHandCommander
^^^^^^^^^^^^^^^^

Overview
~~~~~~~~~
The SrHandCommander inherits all methods from the `robot commander <RobotCommander.html>`__ and provides commands specific to the hand. It allows the state of the tactile sensors and joints' effort to be read, and the maximum force to be set.

Setup
~~~~~~~~~
Import the hand commander along with basic rospy libraries and the hand finder:

.. code-block:: python

    import rospy
    from sr_robot_commander.sr_hand_commander import SrHandCommander
    from sr_utilities.hand_finder import HandFinder
    rospy.init_node("hand_finder_example", anonymous=True)

The constructor for the ``SrHandCommander`` takes a name parameter that should match the group name of the robot to be used. Also it takes the hand prefix, parameters and serial number that can be retrieved using the `HandFinder <https://github.com/shadow-robot/sr_core/blob/indigo-devel/sr_utilities/scripts/sr_utilities/hand_finder.py>`__.

Example
++++++++

.. code-block:: python

    # Using the HandFinder
    hand_finder = HandFinder()
    hand_parameters = hand_finder.get_hand_parameters()
    hand_serial = hand_parameters.mapping.keys()[0]

    # If name is not provided, it will set "right_hand" or "left_hand" by default, depending on the hand.
    hand_commander = SrHandCommander(name = "rh_first_finger",
                                     hand_parameters=hand_parameters,
                                     hand_serial=hand_serial)

    # Alternatively you can launch the hand directly
    hand_commander = SrHandCommander(name = "right_hand", prefix = "rh")

Getting information
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use the ``get_joints_effort`` method to get a dictionary with efforts of the group joints.

.. code-block:: python

    hand_joints_effort = hand_commander.get_joints_effort()
    print("Hand joints effort \n " + str(hand_joints_effort) + "\n")


Use the ``get_tactile_type`` to get a string indicating the type of tactile
sensors present (e.g. PST, biotac, UBI0) or ``get_tactile_state`` to get
an object containing tactile data. The structure of the data is
different for every ``tactile_type`` .

.. code-block:: python

    tactile_type = hand_commander.get_tactile_type()
    tactile_state = hand_commander.get_tactile_state()

    print("Hand tactile type\n" + tactile_type + "\n")
    print("Hand tactile state\n" + str(tactile_state) + "\n")

Set the maximum force
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use the method ``set_max_force`` to set the maximum force for a hand joint.

Parameters:

-  *joint\_name* name of the joint.
-  *value* maximum force value

Example
++++++++

.. code-block:: python

    ## The limits in the current implementation of the firmware are from 200 to 1000 (measured in custom units)
    hand_commander.set_max_force("rh_FFJ3", 600)

SrArmCommander
^^^^^^^^^^^^^^^

The SrArmCommander inherits all methods from the `robot commander](https://dexterous-hand.readthedocs.io/en/latest/user_guide/2_software_description.html#srrobotcommander) and provides commands specific to the arm. It allows movement to a certain position in cartesian space, to a configuration in joint space
or move using a trajectory.

Setup
~~~~~~~~~
Import the arm commander along with basic rospy libraries and the arm finder:

.. code-block:: python

    import rospy
    from sr_robot_commander.sr_arm_commander import SrArmCommander
    from sr_utilities.arm_finder import ArmFinder

The constructors for ``SrArmCommander`` take a name parameter that should match the group name of the robot to be used and has the option to add ground to the scene.

.. code-block:: python

   arm_commander = SrArmCommander(name="right_arm", set_ground=True)
   
Use the ArmFinder to get the parameters (such as prefix) and joint names of the arm currently running on the system:

.. code-block:: python

   arm_finder = ArmFinder()
   
   # To get the prefix or mapping of the arm joints. Mapping is the same as prefix but without underscore.
   arm_finder.get_arm_parameters().joint_prefix.values()
   arm_finder.get_arm_parameters().mapping.values()
   
   # To get the arm joints
   arm_finder.get_arm_joints()

Getting basic information
~~~~~~~~~~~~~~~~~~~~~~~~~~~
To return the reference frame for planning in cartesian space:

.. code-block:: python

   reference_frame = arm_commander.get_pose_reference_frame()

Plan/move to a position target
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using the method ``move_to_position_target``, the end effector of the arm can be moved to a certain point
in space represented by (x, y, z) coordinates. The orientation of the end effector can take any value.

Parameters:

-  *xyz* desired position of end-effector
-  *end\_effector\_link* name of the end effector link (default value is
   empty string)
-  *wait*  indicates if the method should wait for the movement to end or not
   (default value is True)

Example
++++++++

.. code-block:: python

   rospy.init_node("robot_commander_examples", anonymous=True)
   arm_commander = SrArmCommander(name="right_arm", set_ground=True)

   new_position = [0.25527, 0.36682, 0.5426]
    
   # To only plan
   arm_commander.plan_to_position_target(new_position)
    
   # To plan and move
   arm_commander.move_to_position_target(new_position)

Plan/move to a pose target
~~~~~~~~~~~~~~~~~~~~~~~~~~~

Using the method ``move_to_pose_target`` allows the end effector of the arm to be moved to a certain pose
(position and orientation) in the space represented by (x, y, z, rot\_x,
rot\_y, rot\_z).

Parameters:

-  *pose* desired pose of end-effector: a Pose message, a PoseStamped
   message or a list of 6 floats: [x, y, z, rot\_x, rot\_y, rot\_z] or a
   list of 7 floats [x, y, z, qx, qy, qz, qw]
-  *end\_effector\_link* name of the end effector link (default value is
   empty string)
-  *wait* indicates if the method should wait for the movement to end or not
   (default value is True)

Example
++++++++

.. code-block:: python

   rospy.init_node("robot_commander_examples", anonymous=True)
   arm_commander = SrArmCommander(name="right_arm", set_ground=True)

   new_pose = [0.5, 0.3, 1.2, 0, 1.57, 0]
   
   # To only plan
   arm_commander.plan_to_pose_target(new_pose)
   
   # To plan and move
   arm_commander.move_to_pose_target(new_pose)
