Hand Data
----------

Accessing Data from the Hand
----------------------------

There are three main ways to access data from the hand:

* :doc:`/user_guide/sd_graphical_ui` 
* :doc:`/user_guide/sd_command_line_ui`
* :doc:`/user_guide/sd_application_programming_interface`

Examples of each can be found on the above pages. 

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




Saving Robot States
-------------

To save a state you must first be connected to the warehouse. After launching the hand, click the green **Connect** button in the 'Context' tab of rviz.

.. image:: ../img/sd_rviz_warehouse_connect.png

If you have connected successfully you should see two new buttons, **Reset database** and **Disconnect**, as can be seen in the following picture:

.. image:: ../img/sd_rviz_warehouse_connected.png

Next, go to the 'Stored States' tab in 'Motion Planning'. Here you have full control over the saved states in the warehouse. You can then follow these steps:

* move the hand to the grasp position
* Go to the 'Planning' tab and in the 'Select Goal State' select 'current' and click **update**.

.. image:: ../img/sd_rviz_select_goal_state.png

* Finally, go to the 'Stored States' tab and click the button **Save Goal** under the 'Current State' group. A prompt will appear to ask you to name the state. Once named, you can plan to and from this state.

.. image:: ../img/sd_save_state.png
