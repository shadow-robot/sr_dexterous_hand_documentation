.. _installation_instructions:

Installation instructions
=========================

Prerequisites for training and RL model export
----------------------------------------------

Clone the following repos to the following locations:

.. code-block:: bash

    cd ~/
    git clone https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs.git
    cd OmniIsaacGymEnvs
    git clone git@github.com:shadow-robot/sr_hand_control_sandbox.git
    git clone https://github.com/deepmind/mujoco_menagerie/


We're going to use the OmniIsaacGymEnvs docker environment:

.. code-block:: bash

    cd ~/OmniIsaacGymEnvs
    docker/run_docker_viewer.sh


All following terminal commands will be run from inside the docker container. 
To open an additional terminal and connect it to the container, start a new terminal and run:

.. code-block:: bash

    docker exec -it isaac-sim-oige bash


.. _shadow_teleop_container_installation:


Prerequisites for inference on real hardware
--------------------------------------------

You will need a shadow dexterous-hand container, HTC vive hardware and SteamVR installed on your host machine. The vive hardware 
is optional, if you have a different tracking system available then feen free to integrate this instead.


SteamVR
^^^^^^^

You'll need to install SteamVR on your host machine (outside the docker container). You can download it from here: https://cdn.cloudflare.steamstatic.com/client/installer/steam.deb

.. code-block:: bash

    cd ~/Doewnloads
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


Shadow Dexterous Hand container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You will need a Shadow Robot container for rl inference on real hardware. You can set this up with the following oneliner command:

.. code-block:: bash

    bash <(curl -Ls https://raw.githubusercontent.com/shadow-robot/aurora/v2.2.4.2/bin/run-ansible.sh) docker_deploy --branch v2.2.4.2 \
        product=hand_e \
        reinstall=true \
        container_name="rl_inference_real_hw" \
        demo_icons=false


Once this has run, you can start the container with:

.. code-block:: bash

    docker start rl_inference_real_hw


After a few seconds a graphical terminator (terminal) GUI should start


Installing the vive_ros package in the container
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once the container has started, clone the vive_ros package into the ROS workspace in the container:

.. code-block:: bash
    
    cd /home/user/projects/shadow_robot/base/src
    git clone https://github.com/robosavvy/vive_ros.git


Then, follow the OpenVR installation instructions from the 
`Download and build Valve's OpenVR SDK (most recently tested version) <https://github.com/robosavvy/vive_ros/tree/master#download-and-build-valves-openvr-sdk-most-recently-tested-version>`_ section

Now, run catkin_make from the catkin workspace:

.. code-block:: bash

    cd /home/user/projects/shadow_robot/base
    catkin_make


With the vive powered on and connected and SteamVR started on the host machine, run the following command inside the container to start the vive_ros node:

.. code-block:: bash

    rosrun vive_ros vive_node

