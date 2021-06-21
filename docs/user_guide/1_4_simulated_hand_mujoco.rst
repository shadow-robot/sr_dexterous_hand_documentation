Mujoco
=======

`Mujoco <http://www.mujoco.org/>`_ is a robot simulator that has now been adopted by a wide community of researchers and developers, specially for
machine learning applications. We have developed the tools and the model of our dexterous hand to use Mujoco as an alternative to Gazebo. 
Mujoco is not free so follow the next instructions if you have already a `Mujoco License <https://www.roboti.us/license.html>`_.


Obtaining the mujoco simulation
------------------------------

The software is most easily obtained by downloading and running our docker images. Which image you should use depends on whether your host machine has an Nvidia GPU.

Run the following command to pull the docker image:

.. prompt:: bash $

   docker pull shadowrobot/dexterous-hand:melodic-mujoco-v0.0.2

Non-Nvidia GPU systems
^^^^^^^^^^^^^^^^^^^^^^^

Then use this to run the docker container for the first time:

.. prompt:: bash $

   docker run --name mujoco_container -it -e DISPLAY -e LOCAL_USER_ID=$(id -u) -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw --net=host --privileged shadowrobot/dexterous-hand:melodic-mujoco-v0.0.2

Nvidia GPU systems
^^^^^^^^^^^^^^^^^^^^^^^

If you have Nvidia GPU, use following command instead:


.. prompt:: bash $

   docker run -it --name mujoco_container --net=host --privileged -e DISPLAY -e QT_X11_NO_MITSHM=1 --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw shadowrobot/dexterous-hand:melodic-mujoco-v0.0.2


Running the Mujoco Simulation
------------------------------

Inside the container, put your Mujoco key in `/home/user/mjpro150/bin/mjkey.txt`

The easiest way is to just open the file inside of the container using "vim" and paste the contents of the key there.

You could also use `docker cp`, on your host machine terminal:

.. prompt:: bash $

   docker cp <path to your mujoco key file> mujoco_container:/home/user/mjpro150/bin/mjkey.txt

You can then start the simulation of the hand by running the following in the docker container terminal:

.. prompt:: bash $

   roslaunch sr_robot_launch srhand_mujoco.launch

By default, this will launch a right Dexterous Hand Plus. You can also launch a left hand by appending `hand_id:=lh`:

.. prompt:: bash $

   roslaunch sr_robot_launch srhand_mujoco.launch hand_id:=lh

You can also launch a non-Plus Dexterous Hand by appending `hand_type:=hand_e`:

.. prompt:: bash $

   roslaunch sr_robot_launch srhand_mujoco.launch hand_type:=hand_e

These arguments can be combined to launch a non-Plus left Dexterous Hand.

For arm plus hand simulation (ur10 + right Dexterous Hand Plus at the moment) run the following:

.. prompt:: bash $

   roslaunch sr_robot_launch sr_ur_arm_mujoco.launch

Re-Using your Mujoco Container
------------------------------

After stopping your container (in order to shut down your machine, for example), you can re-use the same container by running:

.. prompt:: bash $

   docker start mujoco_container

This will start the container and connect you to the container terminal again. You can run the same roslaunch command as above to start the simulation again.
