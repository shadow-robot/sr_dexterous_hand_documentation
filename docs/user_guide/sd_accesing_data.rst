Accessing Data From The Hand
=============================

There are four main ways to access data from the hand:

* :doc:`/user_guide/sd_user_interface`
* :doc:`/user_guide/sd_command_line`
* :doc:`/user_guide/sd_robot_commander`
* Using `rospy <http://wiki.ros.org/rospy>`_ or `roscpp <http://wiki.ros.org/roscpp>`_

Example: accessing joint state data

* Using the graphical user interface to view the joint state data in the Data Visualizer.
* Using the Command line interface to view the joint state data in the topic `/joint_state`
* Using SrHandCommander methods of:

  * `current_state = hand_commander.get_current_state()`
  * `joints_position = hand_commander.get_joints_position()`
  * `joints_velocity = hand_commander.get_joints_velocity()`
  
* Using `ROS Python subscriber <https://github.com/shadow-robot/sr_interface/blob/noetic-devel/sr_example/scripts/sr_example/advanced/sr_subscriber_example.py>`_ 
  or `ROS CPP subscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>`_

Recording ROS Bags
------------------

A rosbag or bag is a file format in ROS for storing ROS message data. These bags are often created by subscribing to one or more ROS topics, and storing the received message data in an efficient file structure.

The different ways to record and playback ROS bags can be found `here <http://wiki.ros.org/rosbag>`_

Example: Recording and playing a ROS Bag of joint states
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To record a ROS Bag of the /joint_states topic for 1 minute and name it `joint_state_bag.bag`. The `command-line tool <http://wiki.ros.org/rosbag/Commandline>`_ can be used:

.. code-block:: bash

  rosbag record --duration=1m joint_state_bag.bag /joint_states

To find information about the rosbag `joint_state_bag.bag`:

.. code-block:: bash

  rosbag info joint_state_bag.bag

To play back this ROS Bag:

.. code-block:: bash

  rosbag play joint_state_bag.bag

The rosbag command-line has many options of how to record and playback various topics that are published, these can be found `here <http://wiki.ros.org/rosbag/Commandline>`_.

Copying data out of the dexterous hand container
--------------------------------------------------

`docker cp` is a way to copy files/folders between a container and the local filesystem. An extended description can be found `here <https://docs.docker.com/engine/reference/commandline/cp/>`_.

Coping FROM the container TO the file system:

.. code-block:: bash

  docker cp [OPTIONS] CONTAINER:SRC_PATH DEST_PATH

Copying FROM the file system TO the container:

.. code-block:: bash

  docker cp [OPTIONS] DEST_PATH CONTAINER:SRC_PATH

Some of the `[OPTIONS]` include:

+-----------------------------------+------------------------------------------------------------+
|      Name, shorthand              |                 Description                                |
+===================================+============================================================+
| --archive , -a                    |      Archive mode (copy all uid/gid information)           |
+-----------------------------------+------------------------------------------------------------+
| --follow-link , -L                |      Always follow symbol link in SRC_PATH                 |
+-----------------------------------+------------------------------------------------------------+
