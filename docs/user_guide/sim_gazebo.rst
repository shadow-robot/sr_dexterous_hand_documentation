Gazebo
=======

`Gazebo <http://gazebosim.org/>`_ is our default simulator. Follow the instructions on the next section to install and run a simulation of our robot hands using Gazebo.


Starting a robot simulation
------------------------------

The simulation of the system you have can be launched using the simulation icons found in a folder called "Simulation" in the "Shadow Advanced Launchers" folder. Alternatively you can use the below commands in a terminal:

First you need to start the hand container by either double clicking the icon ``1 - Launch Server Container`` in the "Shadow Advanced Launchers" folder.

Shadow Dexterous hands
^^^^^^^^^^^^^^^^^^^^^^^

* To start the simulation, you can run:

  .. code-block:: shell

     roslaunch sr_robot_launch srhand.launch sim:=true 

* If it is a left hand, ``side:=left`` should be added. For example:

  .. code-block:: shell

     roslaunch sr_robot_launch srhand.launch sim:=true side:=left

* Moveit will enable advanced behaviour (inverse kinematics, planning, collision detectection, etc...), but if it is not needed, you can set ``use_moveit:=false``

Bimanual hand system
^^^^^^^^^^^^^^^^^^^^

.. image:: ../img/sim_bimanual.png
    :align: center
    :alt: Bimanual

To start the simulation of a bimanual system, you can run:

.. code-block:: shell

   roslaunch sr_robot_launch sr_bimanual.launch sim:=true
   
Unimanual arm and hand system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. image:: ../img/sim_unimanual_arm_and_hand.png
    :align: center

To start the simulation of a unimanual right system, you can run:

.. code-block:: shell

   roslaunch sr_robot_launch sr_right_ur10arm_hand.launch sim:=true
  
To add a scene, you can add ``scene:=true`` and you our default scene. You can also add your own scene adding a ``scene_file`` parameter.

.. image:: ../img/sim_unimanual_arm_and_hand_with_scene.png
    :align: center

Similarly, to start the simulation of a unimanual left system, you can run:

.. code-block:: shell

   roslaunch sr_robot_launch sr_left_ur10arm_hand.launch

Bimanual arm and hand system
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. image:: ../img/sim_bimanual_arm_and_hand.png
    :align: center

To start the simulation of a bimanual arm and hand system, you can run:

.. code-block:: shell

   roslaunch sr_robot_launch sr_bimanual_ur10arms_hands.launch external_control_loop:=false sim:=true arm_speed_scale:=0.7 scene:=true
  
To add a scene, you can add ``scene:=true`` and you will see our default scene. 

.. image:: ../img/sim_bimanual_arm_and_hand_with_scene.png
    :align: center

You can also add your own scene adding a ``scene_file`` parameter.
    
Installing the simulator in a different computer
--------------------------------------------------

Follow these instructions if do not have a real hand but would like to use our hand in simulation or you want to install only the simulator on a different computer.

* ROS Noetic:

  .. code-block:: shell

     bash <(curl -Ls https://raw.githubusercontent.com/shadow-robot/aurora/v2.2.2.1/bin/run-ansible.sh) docker_deploy --branch v2.2.2.1 tag=noetic-v1.0.27 product=hand_e nvidia_docker=true reinstall=true sim_icon=true container_name=dexterous_hand_simulated

.. table::
   :class: tight-table
   
   +------------------------+------------------------------------+----------------------------------------------------------------------+
   | Additional parameter   | Values                             | Description                                                          |
   +========================+====================================+======================================================================+
   | product                | hand_e, hand_lite, hand_extra_lite | Describes the shadow hand product you want to install.               |
   +------------------------+------------------------------------+----------------------------------------------------------------------+
   | reinstall              | true, **false**                    | Flag to know if the docker container should be fully reinstalled.    |
   +------------------------+------------------------------------+----------------------------------------------------------------------+
   | nvidia_docker          | true, **false**                    | Define if nvidia-container-toolkit is used. Use with nvidia GPU.     |
   +------------------------+------------------------------------+----------------------------------------------------------------------+
   | launch_hand            | true, **false**                    | Specify if hand driver should start when double clicking desktop icon|
   +------------------------+------------------------------------+----------------------------------------------------------------------+
   | sim_hand               | true, **false**                    | If true the icon's will autolaunch hand in simulation mode.          |
   +------------------------+------------------------------------+----------------------------------------------------------------------+
   | hand_side              | **right**, left                    | Specify if the hand is right or left (ignored if bimanual=true)      |
   +------------------------+------------------------------------+----------------------------------------------------------------------+
   | bimanual               | true, **false**                    | Specify if both hands are used or not.                               |
   +------------------------+------------------------------------+----------------------------------------------------------------------+

You can tell if the installation via the one-liner was successful based on it returnining:

.. code-block:: shell

   Operation completed

The one-liner will then create a dekstop icon that you can open and use to launch the container. If you did not have the parameter ``launch_hand=true`` in your one-liner then you can use the commands shown at the top of this page to launch the simulated hand.

More params and their explanation can be found `here. <https://github.com/shadow-robot/aurora/blob/v2.1.6/ansible/inventory/local/group_vars/docker_deploy.yml>`_
