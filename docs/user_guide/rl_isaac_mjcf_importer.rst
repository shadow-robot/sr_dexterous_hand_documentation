The Isaac Sim MJCF importer
=======================


Overview
--------


Geom names
----------

Internally, the Isaac MJCF importers default name for a geom is: ``_``. For one geom per body, this isn't a problem (due to the nested structure of the resulting .usda file). However, when there is more than one geom per body, each geom will overwrite the the previous one within the same body. We can get around this by specifying a name for each geom when there is more than one geom per body.

For example, in the `mujoco_menagerie <https://github.com/shadow-robot/mujoco_menagerie/tree/main>`_ `right shadow hand <https://github.com/shadow-robot/mujoco_menagerie/blob/main/shadow_hand/right_hand.xml>`_, we can see that the `wrist body <https://github.com/shadow-robot/mujoco_menagerie/blob/1afc8be64233dcfe943b2fe0c505ec1e87a0a13e/shadow_hand/right_hand.xml#L104-L118>`_ contains multiple geoms:

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



Mesh names
----------

Scale
-----

Joint limits
------------

Controller settings
-------------------

