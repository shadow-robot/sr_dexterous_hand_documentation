Running the exported policy on a real hand
==========================================


Start the container created in the :ref:`shadow_container_installation` section and wait a few seconds for the terminator window to open:

.. code-block:: bash

    docker start rl_inference_real_hw


Now, copy the exported policy directory into the container, replacing `your_experiment_name` with the directory containing your model 
(execute this command outside the docker container):

.. code-block:: bash

    docker cp ~/OmniIsaacGymEnvs/omniisaacgymenvs/runs/your_experiment_name rl_inference_real_hw:/home/user

The rest of the commands in this section should be executed inside this window. 
If you want additional terminals, right click on the terminator window and select "Split Horizontally" or "Split Vertically".


Clone this repository to the following location inside the container:

.. code-block:: bash

    cd /home/user/projects/shadow_robot/base/src
    git clone git@github.com:shadow-robot/sr_reinforcement_learning_toolbox.git
    cd /home/user/projects/shadow_robot/base
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo


@TODO: change this to not need private code, and/or link vive setup instructions

Connect your vive, and in a host terminal (outside the docker containers) run the following command to start the steamvr vive interface:

.. code-block:: bash

    cd /home/user/.steam/bin/ && STEAM_RUNTIME=1 ./vrstartup.sh


Now, in seperate terminals (all in the `rl_inference_real_hw` container), run the following commands to start the hand and run the RL policy:

.. code-block:: bash

    roslaunch sr_reinforcement_learning_toolbox sr_rl_vive_tracker_reorientation.launch
    rosrun sr_reinforcement_learning_toolbox sr_vive_reorientation_real_hw.py --model_path /home/user/your_experiment_name/nn/your_experiment_name.onnx

