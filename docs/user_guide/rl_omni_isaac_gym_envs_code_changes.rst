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


tasks/shadow_hand.py
--------------------

We need to add the following helpful variables for later in this file. We should add these to the `ShadowHandTask __init__(...) method, here <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shadow_hand.py#L72C1-L72C1>`_:

.. code-block:: python

    self.hand_name = 'right_hand'
    side = f'{self.hand_name[0]}h'
    self._hand_joint_prefix = f'{side}_'

After that has been done, we'll update the `self.fingertips list <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shadow_hand.py#L74-L80>`_ 
which will become:

.. code-block:: python

    self.fingertips = [
        f"{self._hand_joint_prefix}ffdistal",
        f"{self._hand_joint_prefix}mfdistal",
        f"{self._hand_joint_prefix}rfdistal",
        f"{self._hand_joint_prefix}lfdistal",
        f"{self._hand_joint_prefix}thdistal",
    ]

If we want to add additional objects to the simulation (so that we can learn to manipulate them), we need to 
add them to the acceptable objects assertion. For now, we will allow an object called `vive`, so `this assert line <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shadow_hand.py#L55>`_ 
will become:

.. code-block:: python

            assert self.object_type in ["block", "vive"]

For some reason I've `added a variable here <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shadow_hand.py#L98>`_ 
changing that line to this:

.. code-block:: python

    self.pose_dx, self.pose_dy, self.pose_dz = 0.39, 0, 0.10

This doesn't sound right, CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING 
CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING 
CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING 
CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING CHECK THIS BEFORE PUBLISHING 

To reflect the different default hand name in the imported MJCF file 
we need to update the prim path when we initialize the class modified in this section: `robots/articulations/shadow_hand.py`_, 
as well as the name of the hand that the articulation settings are applied to. This means that `this section <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shadow_hand.py#L101-L111>`_ 
will become:

.. code-block:: python

    shadow_hand = ShadowHand(
        prim_path=self.default_zero_env_path + f"/{self.hand_name}",
        name=self.hand_name,
        translation=self.hand_start_translation,
        orientation=self.hand_start_orientation,
    )
    self._sim_config.apply_articulation_settings(
        f"{self.hand_name}",
        get_prim_at_path(shadow_hand.prim_path),
        self._sim_config.parse_actor_config(f"{self.hand_name}"),
    )

Similarly, we need to change the hand name passed into the class modified in this section: `robots/articulations/views/shadow_hand_view.py`_, 
`from this <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shadow_hand.py#L116>`_, 
to this:

.. code-block:: python

    prim_paths_expr = f"/World/envs/.*/{self.hand_name}/{self._hand_joint_prefix}forearm"
    hand_view = ShadowHandView(prim_paths_expr=prim_paths_expr, name="shadow_hand_view")


.. note::
    The default Isaac MJCF importer can generate unstable hand models. These instabilities often result in `NAN`s turning up 
    in the observation space tensor, in locations corresponding to the hand variables linked to the simulation instability. For this reason, 
    while not essential, it can be useful to check for `nan`s in the observation space. We can do this by adding the following code before the `return observations` 
    statement, in the `get_observations(self):.. method, here <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shadow_hand.py#L152>`_

    .. code-block:: python
        
        nans = torch.isnan(self.obs_buf).nonzero()
        if nans.shape[0] != 0:
            print('NaN(s) detected in input observation space, this is generally caused by simulation instabilities. '
                  'Please check your input HAND_MODEL file')
            print(f'obs nans: {nans}')
            for x in nans:
                print(x)


tasks/shared/in_hand_manipulation.py
------------------------------------

There are two sets of changes we make to this file.

The first lets us load in a custom object (we use a HTC vive tracker).

The second, is the addition of a hold count buffer. By default, the reward function registers a success 
whenever the object passes within the goal orientation. The hold count buffer slightly modifies this behaviour, 
so that the reward can only register a success if the object is held at the target orientation for a number of simulation steps.
We found that this improves the sim2real performance of the learned policy.

To add the path for the custom (vive) object, change `this line in the get_object method <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py#L195>`_ 
to this:

.. code-block:: python

    if self.object_type == "block":
        self.object_usd_path = f"{self._assets_root_path}/Isaac/Props/Blocks/block_instanceable.usd"
    else:
        self.object_usd_path = '/workspace/omniisaacgymenvs/sr_assets/objects/test_vive_2_flat.usda'


To add the hold count buffer, we need to add a new variable to the `InHandManipulationTask __init__(...) method, here <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py#L53>`_

 .. code-block:: python

    self.hold_count_buf = self.progress_buf.clone()


We also need to fetch the number of successive simulation steps we want to hold the object for to the `update_config(self) method <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py#L76>`_:

.. code-block:: python

    self.num_success_hold_steps = self._task_cfg["env"].get("num_success_hold_steps", 1)


The `self.hold_count_buf` buffer should be reset for any environments that are reset, so we need to add it to the `reset_idx(self, env_ids):.. method <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py#L438>`_

.. code-block:: python

    self.hold_count_buf[env_ids] = 0


The `self.hold_count_buf` buffer should be passed into the reward function and returned, so we need to add it to where we call `compute_hand_reward` in the `calculate_metrics(self)` method. 
So, `these lines here <https://github.com/shadow-robot/OmniIsaacGymEnvs/blob/f0f0c3d7e222b7d787182ed19ec162c2a3e6ec4e/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py#L271-L304>`_ 
will become:

.. code-block:: python

    def calculate_metrics(self):
        (
            self.rew_buf[:],
            self.reset_buf[:],
            self.reset_goal_buf[:],
            self.progress_buf[:],
            self.hold_count_buf[:],
            self.successes[:],
            self.consecutive_successes[:],
        ) = compute_hand_reward(
            self.rew_buf,
            self.reset_buf,
            self.reset_goal_buf,
            self.progress_buf,
            self.hold_count_buf,
            self.successes,
            self.consecutive_successes,
            self.max_episode_length,
            self.object_pos,
            self.object_rot,
            self.goal_pos,
            self.goal_rot,
            self.dist_reward_scale,
            self.rot_reward_scale,
            self.rot_eps,
            self.actions,
            self.action_penalty_scale,
            self.success_tolerance,
            self.reach_goal_bonus,
            self.fall_dist,
            self.fall_penalty,
            self.max_consecutive_successes,
            self.av_factor,
            self.num_success_hold_steps,
        )

We also need to change the signature of the reward function to add the `self.hold_count_buf`. `The reward function input arguments <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py#L457-L480>`_ 
should be changed to this:

.. code-block:: python

    def compute_hand_reward(
        rew_buf,
        reset_buf,
        reset_goal_buf,
        progress_buf,
        hold_count_buf,
        successes,
        consecutive_successes,
        max_episode_length: float,
        object_pos,
        object_rot,
        target_pos,
        target_rot,
        dist_reward_scale: float,
        rot_reward_scale: float,
        rot_eps: float,
        actions,
        action_penalty_scale: float,
        success_tolerance: float,
        reach_goal_bonus: float,
        fall_dist: float,
        fall_penalty: float,
        max_consecutive_successes: int,
        av_factor: float,
        num_success_hold_steps: int,
    ):

And the `variables that the reward function returns <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py#L531>`_ 
should be changed to this:

.. code-block:: python
    
    return reward, resets, goal_resets, progress_buf, hold_count_buf, successes, cons_successes

Finally, we need to change the reward function itself to utilise the new `hold_count_buf`. To do that, we replace `this goal_resets = ... <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/tasks/shared/in_hand_manipulation.py#L499>`_ 
line with the following:

.. code-block:: python

    goal_reached = torch.where(torch.abs(rot_dist) <= success_tolerance, torch.ones_like(reset_goal_buf), reset_goal_buf)
    hold_count_buf = torch.where(goal_reached == 1, hold_count_buf + 1, torch.zeros_like(goal_reached))
    goal_resets = torch.where(hold_count_buf > num_success_hold_steps, torch.ones_like(reset_goal_buf), reset_goal_buf)


cfg/task/ShadowHandOpenAI_FF.yaml
---------------------------------

Finally, we need to apply some of the functionality we've added above. Currently, the best task config for sim2real transfer is the `ShadowHandOpenAI_LSTM.yaml config <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/main/omniisaacgymenvs/cfg/task/ShadowHandOpenAI_LSTM.yaml>`_.
That file just changes the default number of environments and then calls this the `ShadowHandOpenAI_FF.yaml file <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/main/omniisaacgymenvs/cfg/task/ShadowHandOpenAI_FF.yaml>`_
, so that is where we will make our changes.

Lets add the `num_success_hold_steps` variable to the `env section of the config file <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/cfg/task/ShadowHandOpenAI_FF.yaml#L28>`_. This will be the number of successive simulation steps we want to hold the object for to register a success.

.. code-block:: yaml

    num_success_hold_steps: 10

We also need to change the object from "block" to "vive". So replace `this objectType line <https://github.com/NVIDIA-Omniverse/OmniIsaacGymEnvs/blob/8cf773ab6cac0c8e0d55f46d6d7d258e781c6458/omniisaacgymenvs/cfg/task/ShadowHandOpenAI_FF.yaml#L45>`_ 
with this:

.. code-block:: yaml

    objectType: "vive"