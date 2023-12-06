Installation instructions
=========================

Prerequisites for training and RL model export
----------------------------------------------

.. _isaac_repo_installation:

Cloning Repositories
^^^^^^^^^^^^^^^^^^^^

Clone the following repos to the following locations:

.. code-block:: bash

    cd ~/
    git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs.git
    cd OmniIsaacGymEnvs
    git clone git@github.com:shadow-robot/sr_reinforcement_learning_toolbox.git
    git clone https://github.com/deepmind/mujoco_menagerie/


.. _isaac_container_installation:

Creating the Isaac Sim container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
We're going to use the OmniIsaacGymEnvs docker environment:

.. code-block:: bash

    cd ~/OmniIsaacGymEnvs
    docker/run_docker_viewer.sh


.. note::

    This container will not persist over a reboot. You can delete and re-create this container, any changes inside it 
    are mapped to directories outside the container, so you won't loose any work.


Prerequisites for inference on real hardware
--------------------------------------------

You will need a shadow dexterous-hand container, HTC vive hardware and SteamVR installed on your host machine. The vive hardware 
is optional, if you have a different tracking system available then feen free to integrate that instead.


SteamVR
^^^^^^^

You'll need to install SteamVR on your host machine (outside the docker container). You can download it from here: https://cdn.cloudflare.steamstatic.com/client/installer/steam.deb

.. code-block:: bash

    cd ~/Downloads
    sudo dpkg -i steam_latest.deb


Depending on the output of the above command, you may need to run:

.. code-block:: bash

    sudo apt-get install --fix-broken
    sudo apt-get install --fix-missing


Then, start steam:

.. code-block:: bash

    steam


Follow the instructions to install the required packages and start steam. Then, from the steam interface, 
search for the application `SteamVR` and install it. Connect your vive hardware and start SteamVR.


.. _shadow_container_installation:

Creating the Shadow Dexterous Hand container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You will need a Shadow Robot container for rl inference on real hardware. You can set this up with the following oneliner command:

.. code-block:: bash

    bash <(curl -Ls https://raw.githubusercontent.com/shadow-robot/aurora/v2.2.4.2/bin/run-ansible.sh) docker_deploy --branch v2.2.4.2 \
        product=hand_e \
        reinstall=true \
        container_name="rl_inference_real_hw" \
        demo_icons=false


Once this has finished running, you can start the container with:

.. code-block:: bash

    docker start rl_inference_real_hw


After a few seconds a graphical terminator (terminal) GUI should start.

.. note::
    
    This container will persist after a reboot, you can simply start it again at any time with 
    ``docker start rl_inference_real_hw``.

.. warning:: 

     If you delete this container, any changes inside it will be lost forever.


Installing the vive_ros package in the shadow dexterous hand container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once the container has started, clone the `vive_ros <https://github.com/robosavvy/vive_ros/>`_ package into the ROS workspace in the container:

.. code-block:: bash
    
    cd /home/user/projects/shadow_robot/base/src
    git clone https://github.com/robosavvy/vive_ros.git


Then, follow the OpenVR installation instructions from the 
`Download and build Valve's OpenVR SDK (most recently tested version) <https://github.com/robosavvy/vive_ros/tree/master#download-and-build-valves-openvr-sdk-most-recently-tested-version>`_ section.

Now, run catkin_make from the catkin project workspace:

.. code-block:: bash

    cd /home/user/projects/shadow_robot/base
    catkin_make


With the vive powered on and connected, and SteamVR started on the host machine, run the following command inside the container to start the vive_ros node:

.. code-block:: bash

    rosrun vive_ros vive_node



Using the docker containers
---------------------------

All following terminal commands will be run from inside one of the two docker containers created above.

The two containers we have created on this page are called:

* ``isaac-sim-oige`` (for isaac sim GUI, RL training, model export)
* ``rl_inference_real_hw`` (for inference on real hardware with the shadow dexterous hand and a vive tracker)

To start either container, run the following command:

.. code-block:: bash

    docker start <container_name>



Isaac Sim container (for training)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To open a terminal and connect it to the isaac sim container, start a new terminal and run:

.. code-block:: bash

    docker exec -it isaac-sim-oige bash



Dexterous Hand container (for inference on real hardware)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To open a terminal and connect it to the dexterous hand container, start a new terminal and run:

.. code-block:: bash

    docker exec -it rl_inference_real_hw bash
    su user

.. note::
    
    All following terminal commands in this guide should be executed from inside one of these two containers, 
    unless it's explicitlly stated that the command should be executed on the host, in which case open a normal 
    terminal and don't connect to a container.