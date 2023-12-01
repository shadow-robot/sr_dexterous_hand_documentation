Exporting the Isaac RL policy to onnx
=====================================

we will be using a tool called `torch_rlgames_export_cfg.py`. This should be run from inside an isaac container, on the same machine that you 
used to train the RL policy.

To export the Isaac RL policy to onnx, from inside an isaac container go to the following directory 
(should have been cloned in the :ref: `_installation_instructions` section):

.. code-block:: bash

    cd /workspace/omniisaacgymenvs/sr_hand_control_sandbox/sr_rl_vive_reorientation/src/sr_rl_vive_reorientation


You will need to pass in the following arguments to the tool:

.. code-block:: bash
    
    usage: torch_rlgames_export_isaacsim_cfg.py [-h] -c CHECKPOINT

    options:
    -h, --help            show this help message and exit
    -c CHECKPOINT, --checkpoint CHECKPOINT
                            The checkpoint file you wish to export (should end in .pth)

.. note:: 
    This tool expects the checkpoint file structure to be as-is from the training process. 
    More information on this `can be found here <https://github.com/shadow-robot/sr_hand_control_sandbox/tree/F_vive_reorientation/sr_rl_vive_reorientation#torch_rlgames_export_cfgpy>`_.


Then run the following command, replacing the checkpoint file name with the one you wish to export:

.. code-block:: bash
    
    /isaac-sim/python.sh torch_rlgames_export_isaacsim_cfg.py -c /workspace/omniisaacgymenvs/omniisaacgymenvs/runs/your_experiment_name/nn/last_your_experiment_name_ep_9000_rew_6783.5625.pth


This may take a few minutes to run. Once it has finished, you should see a file called `your_checkpoint_filename.onnx` 
in the same directory, along with the following files:

* your_checkpoint_filename.actuated_dof_indices
* your_checkpoint_filename.cfg_dict
* your_checkpoint_filename.dof_index_map
* your_checkpoint_filename.dof_lower
* your_checkpoint_filename.dof_upper
* your_checkpoint_filename.hidden_state
* your_checkpoint_filename.out_state
* your_checkpoint_filename.running_mean_std_mean
* your_checkpoint_filename.running_mean_std_var

These files contain information such as:

* Which degrees of freedom are actuated
* A backup of the training config
* A DOF to joint name map
* Joint limit values
* The state of the RL networks
* Some tensors of values that the training process learned to filter the joints during training (also needed at runtime)

All of these will be automatically loaded by the runtime code.