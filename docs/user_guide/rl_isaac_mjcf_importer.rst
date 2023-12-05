Isaac Sim MJCF importer
=======================


Overview
--------

The Isaac Sim MJCF importer is a tool to convert MJCF models to the pixar `.USD` format (which Isaac Sim uses).

USD MJCF Fixer
--------------

At the time of writing (using isaac-sim:2023.1.0-hotfix.1), we have found that the .USD files that the importer produces do not result in stable simulations. 
A USD/MJCF fixer has been written to fix these models. Instructions on using this fixer can be found here:

`https://github.com/shadow-robot/sr_reinforcement_learning_toolbox/blob/F_SRC-7134_first_rl_example_add_usda_mjcf_fixer/sr_usda_mjcf_fixer/README.md <https://github.com/shadow-robot/sr_reinforcement_learning_toolbox/blob/F_SRC-7134_first_rl_example_add_usda_mjcf_fixer/sr_usda_mjcf_fixer/README.md>`_

Before continuing with this tutorial, please follow the above instructions.
The rest of this tutorial assumes that you have a fixed .USDA file at the following location inside the isaac sim container:

.. code-block:: bash

  /workspace/omniisaacgymenvs/mujoco_menagerie/shadow_hand/right_hand/right_hand_fixed.usda


Joint and tendon names
-----------

The isaac code and default models use a different joint naming convention from both the mujoco_menagerie model and the real shadow hand. We will make changes to the isaac code later to reflect these differences.