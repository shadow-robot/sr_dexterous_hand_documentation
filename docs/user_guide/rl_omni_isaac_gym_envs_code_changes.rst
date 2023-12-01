OmniIsaacGymEnvs code changes
=============================


Overview
--------

This page describes the changes which must be made to the `Nvidia OmniIsaacGymEnvs repository <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs>`_ 
which lets it work with hand models (`.usd(a)` files ) generated from the mujoco menagerie. This has been done because the isaac repo doesn't contain
a left hand model, so the import process must be followed to learn left hand policies.

The files that we will change are:

* `omniisaacgymenvs/robots/articulations/shadow_hand.py <https://github.com/shadow-robot/OmniIsaacGymEnvs/blob/F_testing_2023_hf1/omniisaacgymenvs/robots/articulations/shadow_hand.py>`_
* `omniisaacgymenvs/robots/articulations/views/shadow_hand_view.py <https://github.com/shadow-robot/OmniIsaacGymEnvs/blob/F_testing_2023_hf1/omniisaacgymenvs/robots/articulations/views/shadow_hand_view.py>`_
* `omniisaacgymenvs/tasks/shadow_hand.py <https://github.com/shadow-robot/OmniIsaacGymEnvs/blob/F_testing_2023_hf1/omniisaacgymenvs/tasks/shadow_hand.py>`_
* `omniisaacgymenvs/tasks/shared/in_hand_manipulation.py <https://github.com/shadow-robot/OmniIsaacGymEnvs/blob/F_testing_2023_hf1/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py>`_

And, depending on which policy you want to learn, you will also need to change one of the following:

* `omniisaacgymenvs/cfg/task/ShadowHandOpenAI_FF.yaml <https://github.com/shadow-robot/OmniIsaacGymEnvs/blob/F_testing_2023_hf1/omniisaacgymenvs/cfg/task/ShadowHandOpenAI_FF.yaml>`_
* `omniisaacgymenvs/cfg/task/ShadowHand.yaml <https://github.com/shadow-robot/OmniIsaacGymEnvs/blob/F_testing_2023_hf1/omniisaacgymenvs/cfg/task/ShadowHand.yaml>`_

Note: All of these changes have been made in the `Shadow Robot fork of the OmniIsaacGymEnvs repo <https://github.com/shadow-robot/OmniIsaacGymEnvs/tree/F_testing_2023_hf1>`_


robots/articulations/shadow_hand.py
-----------------------------------

We need to change where Isaac looks for the hand model. This can be found `here <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/robots/articulations/shadow_hand.py#L56-L59>`_

.. code-block:: python

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
    self._usd_path = assets_root_path + "/Isaac/Robots/ShadowHand/shadow_hand_instanceable.usd"


We're using a local file and so don't need the `assets_root_path` anymore.
Also, for later in this file, it will be helpful to create and set a variable for `hand side`. The above code block would simply become:

.. code-block:: python

    self._usd_path = "/workspace/omniisaacgymenvs/mujoco_menagerie/shadow_hand/right_hand/right_hand_fixed.usda"
    self._side = 'rh'


We will need to update the `joints_config` dictionary to match the joints in the hand model. So `these lines <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/robots/articulations/shadow_hand.py#L82-L103>`_ will become:

.. code-block:: python

    side = self._side
    j0_name = 'J2'
    joints_config = {
        f"{side}_WRJ2": {"stiffness": 5, "damping": 0.5, "max_force": 4.785},
        f"{side}_WRJ1": {"stiffness": 5, "damping": 0.5, "max_force": 2.175},
        f"{side}_FFJ4": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_FFJ3": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_FF{j0_name}": {"stiffness": 1, "damping": 0.1, "max_force": 0.7245},
        f"{side}_MFJ4": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_MFJ3": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_MF{j0_name}": {"stiffness": 1, "damping": 0.1, "max_force": 0.7245},
        f"{side}_RFJ4": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_RFJ3": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_RF{j0_name}": {"stiffness": 1, "damping": 0.1, "max_force": 0.7245},
        f"{side}_LFJ5": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_LFJ4": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_LFJ3": {"stiffness": 1, "damping": 0.1, "max_force": 0.9},
        f"{side}_LF{j0_name}": {"stiffness": 1, "damping": 0.1, "max_force": 0.7245},
        f"{side}_THJ5": {"stiffness": 1, "damping": 0.1, "max_force": 2.3722},
        f"{side}_THJ4": {"stiffness": 1, "damping": 0.1, "max_force": 1.45},
        f"{side}_THJ3": {"stiffness": 1, "damping": 0.1, "max_force": 0.99},
        f"{side}_THJ2": {"stiffness": 1, "damping": 0.1, "max_force": 0.99},
        f"{side}_THJ1": {"stiffness": 1, "damping": 0.1, "max_force": 0.81},
    }


Finally, due to a difference in the structure of joints/links in the imported mjcf file, 
`the part of this file which sets up joint drive modes and parameters <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/robots/articulations/shadow_hand.py#L105-L114>`_ 
needs to be changed, and so will become:

.. code-block:: python

    joints_prim_path = f"{self.prim_path}/{self._side}_forearm/joints"
    for joint_name, config in joints_config.items():
        set_drive(
            f"{joints_prim_path}/{joint_name}",
            "angular",
            "position",
            0.0,
            config["stiffness"] * np.pi / 180,
            config["damping"] * np.pi / 180,
            config["max_force"],
        )


robots/articulations/views/shadow_hand_view.py
----------------------------------------------

To reflect the different structure of the joint/link descriptions (and to add a helpful `hand side` variable again), 
we need to change `this code block <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/robots/articulations/views/shadow_hand_view.py#L46-L50>`_ 
to the following:

.. code-block:: python

    self._hand_joint_prefix = 'rh'
    prim_paths_expr = f"/World/envs/.*/right_hand/rh_forearm/{self._side}.*distal"

    self._fingers = RigidPrimView(
        prim_paths_expr="/World/envs/.*/shadow_hand/robot0.*distal",
        prim_paths_expr=prim_paths_expr,
        name="finger_view",
        reset_xform_properties=False,
    )

To reflect the different joint naming convention, we also need to change `these strings <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/robots/articulations/views/shadow_hand_view.py#L59-L80>`_ 
in the `self._actuated_joint_names`` list to:

.. code-block:: python

            j0_name = 'J2'
            self.actuated_joint_names = [
                f"{self._hand_joint_prefix}_WRJ2",
                f"{self._hand_joint_prefix}_WRJ1",
                f"{self._hand_joint_prefix}_FFJ4",
                f"{self._hand_joint_prefix}_FFJ3",
                f"{self._hand_joint_prefix}_FF{j0_name}",
                f"{self._hand_joint_prefix}_MFJ4",
                f"{self._hand_joint_prefix}_MFJ3",
                f"{self._hand_joint_prefix}_MF{j0_name}",
                f"{self._hand_joint_prefix}_RFJ4",
                f"{self._hand_joint_prefix}_RFJ3",
                f"{self._hand_joint_prefix}_RF{j0_name}",
                f"{self._hand_joint_prefix}_LFJ5",
                f"{self._hand_joint_prefix}_LFJ4",
                f"{self._hand_joint_prefix}_LFJ3",
                f"{self._hand_joint_prefix}_LF{j0_name}",
                f"{self._hand_joint_prefix}_THJ5",
                f"{self._hand_joint_prefix}_THJ4",
                f"{self._hand_joint_prefix}_THJ3",
                f"{self._hand_joint_prefix}_THJ2",
                f"{self._hand_joint_prefix}_THJ1",
            ]

Finally, we need to change the names of the fingertips, 
which is done by changing `this line <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/robots/articulations/views/shadow_hand_view.py#L90>`_ 
to this:

.. code-block:: python

    fingertips = [f"{self._hand_joint_prefix}_ffdistal",
                  f"{self._hand_joint_prefix}_mfdistal",
                  f"{self._hand_joint_prefix}_rfdistal",
                  f"{self._hand_joint_prefix}_lfdistal",
                  f"{self._hand_joint_prefix}_thdistal"]
