Accessing Data from the Hand
------------------------------

There are four main ways to access data from the hand:

* :doc:`/user_guide/sd_rqt_plugins`
* :doc:`/user_guide/sd_command_line`
* SrHandCommander (defined in the sections below)
* Using `rospy <http://wiki.ros.org/rospy>`_ or `roscpp <http://wiki.ros.org/roscpp>`_

Example: accessing joint state data
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Using the graphical user interface to view the joint state data in the Data Visualizer.
* Using the Command line interface to view the joint state data in the topic `/joint_state`
* Using SrHandCommander methods of:

  * `current_state = hand_commander.get_current_state()`
  * `joints_position = hand_commander.get_joints_position()`
  * `joints_velocity = hand_commander.get_joints_velocity()`
  
* Using `ROS Python subscriber <https://github.com/shadow-robot/sr_interface/blob/noetic-devel/sr_example/scripts/sr_example/advanced/sr_subscriber_example.py>`_ 
  or `ROS CPP subscriber <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>`_
