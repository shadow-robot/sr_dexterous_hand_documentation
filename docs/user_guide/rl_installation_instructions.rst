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
@TODO: change this

You will need a Shadow Robot container for rl inference on real hardware. You can set this up with the following oneliner command:

.. code-block:: bash

    bash <(curl -Ls https://raw.githubusercontent.com/shadow-robot/aurora/v2.2.4.2/bin/run-ansible.sh) teleop_deploy --branch v2.2.4.2 --inventory simulation \
        reinstall=true \
        image="080653068785.dkr.ecr.eu-west-2.amazonaws.com/shadow-teleop-polhemus-binary" \
        tag="noetic-v0.1.12" \
        real_glove=false \
        use_steamvr=true \
        container_name="rl_inference_real_hw" \
        bimanual=false \
        demo_icons=false

Once this has run, you can start the container with:

.. code-block:: bash

    docker start rl_inference_real_hw

After a few seconds a graphical terminator (terminal) GUI should start