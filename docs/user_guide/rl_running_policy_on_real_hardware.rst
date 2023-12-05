Running the exported policy on a real hand
==========================================


Start the container created in this section :ref:`shadow_container_installation` and wait a few seconds for the terminator window to open:

.. code-block:: bash

    docker start rl_inference_real_hw


Now, copy the exported policy directory into the container, replacing `your_experiment_name` with the directory containing your model 
(execute this outside the docker container):

.. code-block:: bash

    docker cp ~/OmniIsaacGymEnvs/omniisaacgymenvs/runs/your_experiment_name rl_inference_real_hw:/home/user

The rest of the commands in this section should be executed inside this window. 
If you want additional terminals, right click on the terminator window and select "Split Horizontally" or "Split Vertically".


@TODO: change this to not need private repos

@TODO: cleanup branch names

Clone this repository to the following location inside the container:

.. code-block:: bash

    cd /home/user/projects/shadow_robot/base/src
    git clone git@github.com:shadow-robot/sr_hand_control_sandbox.git && cd sr_hand_control_sandbox
    git checkout F_vive_reorientation_add_usd_mjcf_fixer
    cd /home/user/projects/shadow_robot/base && catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo


@TODO: change this to not need private code, and/or link vive setup instructions

Connect your vive, and run the following command to start the steamvr vive interface:

.. code-block:: bash

    cd /home/user/.steam/bin/ && STEAM_RUNTIME=1 ./vrstartup.sh


Now, in seperate terminals (all in the `rl_inference_real_hw` container), run the following commands to start the hand and run the RL policy:

@TODO: add argparse to sr_vive_reorientation_real_hw_2p8.py

@TODO: more sensible filenames, delete unused old code/launch files

.. code-block:: bash

    roslaunch sr_hand_control_sandbox sr_rl_vive_tracker_reorientation_2.launch
    rosrun sr_hand_control_sandbox sr_vive_reorientation_real_hw_2p8.py

