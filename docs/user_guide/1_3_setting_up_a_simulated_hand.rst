------------------------------
Setting up a simulated hand
------------------------------

Gazebo
=======

`Gazebo <http://gazebosim.org/>`_ is our default simultator. Follow the intructions on the next section to install and run a simulation of our robot hands using Gazebo.

Installing the software (sim)
------------------------------

If you do not actually have a real hand but would like to use our hand in simulation, then please run the following command:

* ROS Melodic (Recommended):

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e sim_hand=true launch_hand=true tag=melodic-release

* ROS Kinetic:

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e sim_hand=true launch_hand=true tag=kinetic-release

* ROS Indigo:

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) docker_deploy product=hand_e sim_hand=true launch_hand=true tag=indigo-release


You can also add ``reinstall=true`` true in case you want to reinstall the docker image and container. When it finishes it will show:

.. prompt:: bash $

   Operation completed

and it will create two desktop icons that you can double-click to launch the hand or save the log files from the active containers to your desktop.

If you have an Nvidia graphics card, you can add ``nvidia_docker=true`` to use nvidia-docker.

Starting a robot simulation
------------------------------

First you need to start the hand container by either double clicking the icon "Hand_Container" or running the following command:

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

  +---------+-------------------------+-----------------------+-----------------------+
  |         | Hand                    | hand_type Parameter   | Left Hand Parameter   |
  +=========+=========================+=======================+=======================+
  | |image0|| Hand E                  | hand_e                | hand_id:=lh           |
  +---------+-------------------------+-----------------------+-----------------------+
  | |image1|| Hand E Lite             | hand_lite             | Right hand only       |
  +---------+-------------------------+-----------------------+-----------------------+
  | |image2|| Hand E Extra Lite       | hand_extra_lite       | Right hand only       |
  +---------+-------------------------+-----------------------+-----------------------+
  | |image3|| Hand E Plus             | hand_e_plus           | hand_id:=lh           |
  +---------+-------------------------+-----------------------+-----------------------+

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

Bimanual system
^^^^^^^^^^^^^^^^
.. figure:: ../img/bimanual.png
    :align: center
    :alt: Bimanual


To start the simulation of a bimanual system, you can run:

.. prompt:: bash $

   roslaunch sr_robot_launch sr_bimanual.launch

Mujoco
=======

`Mujoco <http://www.mujoco.org/>`_ is a robot simulator that has now been adopted by a wide community of researchers and developers, specially for
machine learning applications. We have developed the tools and the model of our dexterous hand to use Mujoco as an alternative to Gazebo. 
Mujoco is not free so follow the next instructions if you have already a `Mujoco License <https://www.roboti.us/license.html>`_.


Obtaining the mujoco simulation
------------------------------

The software is most easily obtained by downloading and running our docker images. Which image you should use depends on whether your host machine has an Nvidia GPU.

Non-Nvidia GPU systems
^^^^^^^^^^^^^^^^^^^^^^^

Run the following command to pull the docker image:

.. prompt:: bash $

   docker pull shadowrobot/dexterous-hand:kinetic-mujoco-release

Then use this to run the docker container for the first time:

.. prompt:: bash $

   docker run --name mujoco_container -it -e DISPLAY -e LOCAL_USER_ID=$(id -u) -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw --net=host --privileged shadowrobot/dexterous-hand:kinetic-mujoco-release bash

Nvidia GPU systems
^^^^^^^^^^^^^^^^^^^^^^^

If you have Nvidia GPU, for steps 1 and 2, use following commands instead:

.. prompt:: bash $

   docker pull shadowrobot/dexterous-hand:kinetic-mujoco-release-nvidia

.. prompt:: bash $

   nvidia-docker run --name mujoco_container -it -e DISPLAY -e LOCAL_USER_ID=$(id -u) -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw --net=host --privileged shadowrobot/dexterous-hand:kinetic-mujoco-release-nvidia bash

Note that you will need `nvidia-docker` (version 1) installed. Version 2 support is coming soon.

Running the Mujoco Simulation
------------------------------

Inside the container, put your Mujoco key in `/home/user/mjpro150/bin/mjkey.txt`

The easiest way is to just open the file inside of the container using "vim" and paste the contents of the key there.

You could also use `docker cp`, on your host machine terminal:

.. prompt:: bash $

   docker cp <path to your mujoco key file> mujoco_container:/home/user/mjpro150/bin/mjkey.txt

You can then start the simulation by running the following in the docker container terminal:

.. prompt:: bash $

   roslaunch sr_robot_launch srhand_mujoco.launch

By default, this will launch a right Dexterous Hand Plus. You can also launch a left hand by appending `hand_id:=lh`:

.. prompt:: bash $

   roslaunch sr_robot_launch srhand_mujoco.launch hand_id:=lh

You can also launch a non-Plus Dexterous Hand by appending `hand_type:=hand_e`:

.. prompt:: bash $

   roslaunch sr_robot_launch srhand_mujoco.launch hand_type:=hand_e

These arguments can be combined to launch a non-Plus left Dexterous Hand.

Re-Using your Mujoco Container
------------------------------

After stopping your container (in order to shut down your machine, for example), you can re-use the same container by running:

.. prompt:: bash $

   docker start mujoco_container && docker attach mujoco_container

This will start the container and connect you to the container terminal again. You can run the same roslaunch command as above to start the simulation again.
