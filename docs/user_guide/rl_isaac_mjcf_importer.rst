The Isaac Sim MJCF importer
===========================


Overview
--------


Geom names
----------

Internally, the Isaac MJCF importers default name for a geom is: ``_``\ . For one geom per body, this isn't a problem (due to the nested structure of the resulting .usda file). However, when there is more than one geom per body, each geom defaulting to the same name will overwrite the previous geom in the body. We can get around this by specifying a name for each geom when there is more than one geom per body.

For example, in the `mujoco_menagerie <https://github.com/google-deepmind/mujoco_menagerie>`_ `right shadow hand <https://github.com/google-deepmind/mujoco_menagerie/blob/main/shadow_hand/right_hand.xml>`_, we can see that the `wrist body <https://github.com/google-deepmind/mujoco_menagerie/blob/1afc8be64233dcfe943b2fe0c505ec1e87a0a13e/shadow_hand/right_hand.xml#L104-L118>`_ contains multiple geoms:

.. code-block:: xml

  <geom size="0.0135 0.015" quat="0.499998 0.5 0.5 -0.500002" type="cylinder" class="plastic_collision"/>
  <geom size="0.011 0.005" pos="-0.026 0 0.034" quat="1 0 1 0" type="cylinder" class="plastic_collision"/>
  <geom size="0.011 0.005" pos="0.031 0 0.034" quat="1 0 1 0" type="cylinder" class="plastic_collision"/>
  <geom size="0.0135 0.009 0.005" pos="-0.021 0 0.011" quat="0.923879 0 0.382684 0" type="box" class="plastic_collision"/>
  <geom size="0.0135 0.009 0.005" pos="0.026 0 0.01" quat="0.923879 0 -0.382684 0" type="box" class="plastic_collision"/>

This would produce a ``.usda`` file which only contains the final geom in that body. We can fix this by explicitly naming the geoms:

.. code-block:: xml

  <geom name="rh_wrist_C_0" size="0.0135 0.015" quat="0.499998 0.5 0.5 -0.500002" type="cylinder" class="plastic_collision"/>
  <geom name="rh_wrist_C_1" size="0.011 0.005" pos="-0.026 0 0.034" quat="1 0 1 0" type="cylinder" class="plastic_collision"/>
  <geom name="rh_wrist_C_2" size="0.011 0.005" pos="0.031 0 0.034" quat="1 0 1 0" type="cylinder" class="plastic_collision"/>
  <geom name="rh_wrist_C_3" size="0.0135 0.009 0.005" pos="-0.021 0 0.011" quat="0.923879 0 0.382684 0" type="box" class="plastic_collision"/>
  <geom name="rh_wrist_C_4" size="0.0135 0.009 0.005" pos="0.026 0 0.01" quat="0.923879 0 -0.382684 0" type="box" class="plastic_collision"/>

.. note::
  In the mujoco_menagerie left and right hand models, the bodies containing multiple geoms are:

  * (lh|rh)_forearm

  * (lh|rh)_wrist

  * (lh|rh)_palm

  * (lh|rh)_th_middle

Mesh names
----------

In the mujoco xml specification, a mesh that has not been explicitly named should automatically be given a default name of the file minus the extention. This has not yet been implemented in the isaac MJCF importer, so meshes that have not been explicitly named will crash the importer.

For example, the `mesh definitions for the mujoco menagerie right hand <https://github.com/shadow-robot/mujoco_menagerie/blob/1afc8be64233dcfe943b2fe0c505ec1e87a0a13e/shadow_hand/right_hand.xml#L81-L93>`_:

.. code-block:: xml

  <mesh class="right_hand" file="forearm_0.obj"/>
  <mesh class="right_hand" file="forearm_1.obj"/>
  <mesh class="right_hand" file="forearm_collision.obj"/>
  <mesh class="right_hand" file="wrist.obj"/>
  <mesh class="right_hand" file="palm.obj"/>
  <mesh class="right_hand" file="f_knuckle.obj"/>
  <mesh class="right_hand" file="f_proximal.obj"/>
  <mesh class="right_hand" file="f_middle.obj"/>
  <mesh class="right_hand" file="f_distal_pst.obj"/>
  <mesh class="right_hand" file="lf_metacarpal.obj"/>
  <mesh class="right_hand" file="th_proximal.obj"/>
  <mesh class="right_hand" file="th_middle.obj"/>
  <mesh class="right_hand" file="th_distal_pst.obj"/>

Must all be explicitly named in order to not crash the issac importer:

.. code-block:: xml

  <mesh class="right_hand" name="forearm_0"         file="forearm_0.obj" />
  <mesh class="right_hand" name="forearm_1"         file="forearm_1.obj" />
  <mesh class="right_hand" name="forearm_collision" file="forearm_collision.obj" />
  <mesh class="right_hand" name="wrist"             file="wrist.obj"/>
  <mesh class="right_hand" name="palm"              file="palm.obj"/>
  <mesh class="right_hand" name="f_knuckle"         file="f_knuckle.obj" />
  <mesh class="right_hand" name="f_proximal"        file="f_proximal.obj" />
  <mesh class="right_hand" name="f_middle"          file="f_middle.obj" />
  <mesh class="right_hand" name="f_distal_pst"      file="f_distal_pst.obj" />
  <mesh class="right_hand" name="lf_metacarpal"     file="lf_metacarpal.obj"/>
  <mesh class="right_hand" name="th_proximal"       file="th_proximal.obj" />
  <mesh class="right_hand" name="th_middle"         file="th_middle.obj" />
  <mesh class="right_hand" name="th_distal_pst"     file="th_distal_pst.obj" />


Scale
-----

Joint limits
------------

Controller settings
-------------------

