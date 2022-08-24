Gazebo
=======

`Gazebo <http://gazebosim.org/>`_ is our default simulator. Follow the instructions on the next section to install and run a simulation of our robot hands using Gazebo.

Installing the software (sim)
------------------------------

If you do not actually have a real hand but would like to use our hand in simulation, then please run the following command:

* ROS Noetic (Recommended):

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e sim_hand=true container_name=dexterous_hand_simulated  tag=noetic-release launch_hand=true 
     
* ROS Melodic:

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e sim_hand=true container_name=dexterous_hand_simulated  tag=melodic-release launch_hand=true 

* ROS Kinetic:

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e sim_hand=true container_name=dexterous_hand_simulated  tag=kinetic-release launch_hand=true

* ROS Indigo:

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e sim_hand=true container_name=dexterous_hand_simulated  tag=indigo-release launch_hand=true


You can also add ``reinstall=true`` true in case you want to reinstall the docker image and container. When it finishes it will show:

.. prompt:: bash $

   Operation completed

and it will create two desktop icons that you can double-click to launch the hand or save the log files from the active containers to your desktop.

If you have an Nvidia graphics card, you can add ``nvidia_docker=true`` to use nvidia-docker.

More params and their explanation can be found `here. <https://github.com/shadow-robot/aurora/blob/master/ansible/inventory/local/group_vars/docker_deploy.yml>`_

Starting a robot simulation
------------------------------

First you need to start the hand container by either double clicking the icon ``1 - Launch Server Container`` in the "Shadow Advanced Launchers" folder or running the following command:

.. prompt:: bash $

   docker start dexterous_hand_real_hw


Shadow Dexterous hands
^^^^^^^^^^^^^^^^^^^^^^^
* The hand will start automatically if you have run the one-liner with the argument ``launch_hand=true``. To start it manually, simply run the following command in the container:

  .. prompt:: bash $

     roslaunch sr_robot_launch srhand.launch

  This will launch the five finger hand (shadowhand\_motor) by default .

* If you want to start the dexterous hand plus, you can add the hand\_type like this:

  .. prompt:: bash $

     roslaunch sr_robot_launch srhand.launch hand_type:=hand_e_plus

* If you want to launch another hand, these are the hands available:

  +-------------------------+-----------------------+-----------------------+
  |  Hand                   | hand_type Parameter   | Left Hand Parameter   |
  +=========================+=======================+=======================+
  |  Hand E                 | hand_e                | hand_id:=lh           |
  +-------------------------+-----------------------+-----------------------+
  |  Hand E Lite            | hand_lite             | Right hand only       |
  +-------------------------+-----------------------+-----------------------+
  |  Hand E Extra Lite      | hand_extra_lite       | Right hand only       |
  +-------------------------+-----------------------+-----------------------+
  | Hand E Plus             | hand_e_plus           | hand_id:=lh           |
  +-------------------------+-----------------------+-----------------------+

  .. |image0| image:: ../img/shadowhand_motor.png
  .. |image1| image:: ../img/shadowhand_lite.png
  .. |image2| image:: ../img/shadowhand_extra_lite.png
  .. |image3| image:: ../img/shadowhand_motor_plus.png

  To start the simulation, you can run:

  .. prompt:: bash $

     roslaunch sr_robot_launch srhand.launch hand_type=hand_e

  The ``hand_type`` param can be changed to start any of the available Shadow hands shown in the table.

* If it is a left hand, ``hand_id:=lh`` should be added. For example:

  .. prompt:: bash $

     roslaunch sr_robot_launch srhand.launch hand_type=hand_e_plus hand_id:=lh

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

   roslaunch sr_robot_launch sr_bimanual.launch
   
Unimanual arm and hand system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. figure:: ../img/unimanual_arm_and_hand.png
    :align: center

To start the simulation of a unimanual right system, you can run:

.. prompt:: bash $

   roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
  
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

   roslaunch sr_robot_launch sr_right_ur10arm_hand.launch
  
To add a scene, you can add ``scene:=true`` and you our default scene. You can also add your own scene adding a ``scene_file`` parameter.

.. figure:: ../img/bimanual_arm_and_hand_with_Scene.png
    :align: center
    
