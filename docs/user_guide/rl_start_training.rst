Starting the training
=====================


Check everything is working
---------------------------

Open a new terminal, connect to the isaac docker container and change to the following directory:

.. code-block:: bash

    docker exec -it isaac-sim-oige bash
    cd /workspace/omniisaacgymenvs/omniisaacgymenvs

The vive tracker object uses more VRAM and more of the internal buffers than the default `"block"` object. Therefore, we need to reduce the number of environments. 
We should keep the minibatch_size to 2x the number of environments and the central_value_config.minibatch_size to 4x the number of environments.

First, lets use a small number of environments to check that everything is working properly:

.. code-block:: bash

    num_envs=64 /isaac-sim/python.sh scripts/rlgames_train.py task=ShadowHandOpenAI_LSTM train=ShadowHandOpenAI_LSTMPPO task.env.numEnvs=${num_envs} train.params.config.minibatch_size=$(( $num_envs * 2 )) train.params.config.central_value_config.minibatch_size=$(( $num_envs * 4 ))

After a few minutes you should see the simulation window start, and a grid of 8x8 hands all starting to learn to manipulate the vive tracker. 

Check that the simulation can start without segmentation faults. 

Check that the movements of the hand look normal, no vibrating joints, strange movements, hand joints twisted in the wrong directions etc..

Check that VRAM memory usage stabailises after the first ten minutes (can be checkedd via `nvidia-smi`).

If all of this looks ok, you can start training.

Actual training
---------------

Increase the number of environments to 6100 (the max we have found works on an RTX3090), and add the 
`headless=true` flag to the command line to disable the simulation window. You may also wish to add an `experiment="your_experiment_name_here"` flag 
to the command, to help you keep track of what you're trying in each traning.

.. code-block:: bash

    num_envs=6100 /isaac-sim/python.sh scripts/rlgames_train.py task=ShadowHandOpenAI_LSTM train=ShadowHandOpenAI_LSTMPPO task.env.numEnvs=${num_envs} train.params.config.minibatch_size=$(( $num_envs * 2 )) train.params.config.central_value_config.minibatch_size=$(( $num_envs * 4 )) headless=true experiment="shadow_rl_tutorial"

On our RTX3090 desktop, this takes about 52 hours to reach 10,000 episodes. However, you may find that the reward stops increasing after the first ~36 hours, 
in which case you can stop the training early. Checkpoints are automatically generated and written to 

.. code-block:: bash

    /workspace/omniisaacgymenvs/omniisaacgymenvs/runs/your_experiment_name_here/nn

If you keep an eye on the `.pth` checkpoint files written to that directory, you can get a feel for how well the simulation is going. 
For reference, our reward at the end of a ~52 hour 10,000 episode training is around 7000.