Gazebo
=======

`Gazebo <http://gazebosim.org/>`_ is our default simulator. Follow the instructions on the next section to install and run a simulation of our robot hands using Gazebo.


Starting a robot simulation
------------------------------

First you need to start the hand container by either double clicking the icon ``1 - Launch Server Container`` in the "Shadow Advanced Launchers" folder.


Shadow Dexterous hands
^^^^^^^^^^^^^^^^^^^^^^^


  To start the simulation, you can run:

  .. prompt:: bash $

     roslaunch sr_robot_launch srhand.launch sim:=true 

  

* If it is a left hand, ``side:=left`` should be added. For example:

  .. prompt:: bash $

     roslaunch sr_robot_launch srhand.launch sim:=true side:=left

* Moveit will enable advanced behaviour (inverse kinematics, planning, collision detectection, etc...), but if it is not needed, you can set ``use_moveit:=false``

.. note::
   If when you launch the hand you see some errors related to LibGL, this is a good indication that you have an NVidia card and should add the nvidia flag when running the installation one liner. Run the one liner again with the correct NVidia flags mentioned above and also ``-r true`` to reinstall the docker image and container.

Bimanual hand system
^^^^^^^^^^^^^^^^^^^^
.. figure:: ../img/bimanual.png
    :align: center
    :alt: Bimanual


To start the simulation of a bimanual system, you can run:

.. prompt:: bash $

   roslaunch sr_robot_launch sr_bimanual.launch sim:=true
   
Unimanual arm and hand system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. figure:: ../img/unimanual_arm_and_hand.png
    :align: center

To start the simulation of a unimanual right system, you can run:

.. prompt:: bash $

   roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true
  
To add a scene, you can add ``scene:=true`` and you our default scene. You can also add your own scene adding a ``scene_file`` parameter.

.. figure:: ../img/unimanual_arm_and_hand_with_Scene.png
    :align: center

Similarly, to start the simulation of a unimanual left system, you can run:

.. prompt:: bash $

   roslaunch sr_robot_launch sr_left_ur10arm_hand.launch

Bimanual arm and hand system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. figure:: ../img/bimanual_arm_and_hand.png
    :align: center

To start the simulation of a bimanual arm and hand system, you can run:

.. prompt:: bash $

   roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true
  
To add a scene, you can add ``scene:=true`` and you our default scene. You can also add your own scene adding a ``scene_file`` parameter.

.. figure:: ../img/bimanual_arm_and_hand_with_Scene.png
    :align: center
    
