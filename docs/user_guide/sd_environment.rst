Environment
=================

Robot Operating System (ROS), Linux and Docker
----------------------------------------------

Our systems work within the ROS framework. 

"ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an 
operating system, including hardware abstraction, low-level device control, implementation of commonly-used 
functionality, message-passing between processes, and package management. It also provides tools and libraries for 
obtaining, building, writing, and running code across multiple computers." - ROS.org

If you are unfamiliar with ROS and intend to use the ROS API, you can find the fundamental ROS concepts explained `here <http://wiki.ros.org/ROS/Concepts>`_ and a technical overview of the implementation of ROS `here <http://wiki.ros.org/ROS/Technical%20Overview>`_.
It is highly recommended that you also check the `ROS Tutorials <http://www.ros.org/wiki/ROS/Tutorials>`_.

If you are unfamiliar with Linux and its terminal on Linux, you should look `here <https://ubuntu.com/tutorials/command-line-for-beginners#1-overview>`_.

Shadow software is deployed using Docker. Docker is a container framework where each container image is a lightweight, stand-alone, executable package that includes everything needed to run it. It is similar to a virtual machine but with much less overhead. You can find more information `here <https://www.docker.com/resources/what-container/>`_.

Repositories
------------

Our code is split into different ROS repositories:

* `sr_common <https://github.com/shadow-robot/sr_common>`_: This repository contains the bare minimum for communicating with the Shadow Hand from a remote computer (urdf models and messages).
* `sr_core <https://github.com/shadow-robot/sr_core>`_: These are the core packages for the Shadow Robot hardware and simulation.
* `sr_interface <https://github.com/shadow-robot/sr_interface>`_: This repository contains the high level interface and its dependencies for interacting simply with our robots.
* `sr_tools <https://github.com/shadow-robot/sr_tools>`_: This repository contains more advanced tools that might be needed in specific use cases.
* `sr_visualization <https://github.com/shadow-robot/sr_visualization>`_: This repository contains the various rqt_gui plugins we developed.
* `sr_hand_config <https://github.com/shadow-robot/sr_hand_config>`_: This repository contains the customer specific configuration for the Shadow Robot Hand.

Accessing Data from the Hand
----------------------------

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
^^^^^^^^^^^^^^^^^^

A rosbag or bag is a file format in ROS for storing ROS message data. These bags are often created by subscribing to one or more ROS topics, and storing the received message data in an efficient file structure.

The different ways to record and playback ROS bags can be found `here <http://wiki.ros.org/rosbag>`_

Example: Recording and playing a ROS Bag of joint states
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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
