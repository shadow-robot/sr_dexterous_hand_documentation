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


Connect and power on your vive, and run the following:

.. code-block:: bash
    
    cd ~/
    ./start_steamvr.sh


.. note::
    
    The first time you run ``./start_steamvr.sh`` a web browser will open asking you to accept the SteamVR licence. You must follow the 
    instructions in the browser to accept the licence before you can use SteamVR. You should only have to do this once.

Now, in seperate terminals (all in the `rl_inference_real_hw` container), run the following commands to start the hand and run the RL policy:

.. code-block:: bash

    roslaunch sr_reinforcement_learning_toolbox sr_rl_vive_tracker_reorientation.launch
    rosrun sr_reinforcement_learning_toolbox sr_vive_reorientation_real_hw.py --model_path /home/user/your_experiment_name/nn/your_experiment_name.onnx

