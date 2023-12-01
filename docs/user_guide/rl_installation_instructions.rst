Installation instructions
=========================

Prerequisites
-------------
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
