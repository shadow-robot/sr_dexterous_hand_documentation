Starting a robot simulation
======================================

Shadow hand only
----------------


To start a simulation of our dexterous hand, simply do:

.. code:: bash

    $ roslaunch sr_robot_launch srhand.launch

This will launch the five finger hand (shadowhand\_motor) by default .
If you want to launch another hand, these are the hands available:

+------------+------------+------------+------------+------------+------------+------------+------------+------------+------------+-------------+
| |image0|   | |image1|   | |image2|   | |image3|   | |image4|   | |image5|   | |image6|   | |image7|   | |image8|   | |image9|   | |image10|   |
+============+============+============+============+============+============+============+============+============+============+=============+
| 1          | 2          | 3          | 4          | 5          | 6          | 7          | 8          | 9          | 10         | 11          |
+------------+------------+------------+------------+------------+------------+------------+------------+------------+------------+-------------+

+------+-------------------------------------------------------+----------------------------------------------+
|      | Right                                                 | Left                                         |
+======+=======================================================+==============================================+
| 1    | shadowhand\_motor.urdf.xacro                          | shadowhand\_left\_motor.urdf.xacro           |
+------+-------------------------------------------------------+----------------------------------------------+
| 2    | shadowhand\_motor\_biotac.urdf.xacro                  | shadowhand\_left\_motor\_biotac.urdf.xacro   |
+------+-------------------------------------------------------+----------------------------------------------+
| 3    | shadowhand\_motor\_ff\_biotac.urdf.xacro              |                                              |
+------+-------------------------------------------------------+----------------------------------------------+
| 4    | shadowhand\_motor\_btsp.urdf.xacro                    |                                              |
+------+-------------------------------------------------------+----------------------------------------------+
| 5    | shadowhand\_motor\_ellipsoid.urdf.xacro               |                                              |
+------+-------------------------------------------------------+----------------------------------------------+
| 6    | shadowhand\_motor\_th\_ff\_rf\_ellipsoid.urdf.xacro   |                                              |
+------+-------------------------------------------------------+----------------------------------------------+
| 7    | shadowhand\_muscle.urdf.xacro                         | shadowhand\_left\_muscle.urdf.xacro          |
+------+-------------------------------------------------------+----------------------------------------------+
| 8    | shadowhand\_muscle\_biotac.urdf.xacro                 |                                              |
+------+-------------------------------------------------------+----------------------------------------------+
| 9    | shadowhand\_lite.urdf.xacro                           |                                              |
+------+-------------------------------------------------------+----------------------------------------------+
| 10   | shadowhand\_extra\_lite.urdf.xacro                    |                                              |
+------+-------------------------------------------------------+----------------------------------------------+
| 11   | shadowhand\_motor\_plus.urdf.xacro                    | shadowhand\_left\_motor\_plus.urdf.xacro     |
+------+-------------------------------------------------------+----------------------------------------------+

To start the simulation of a shadow hand, you can run:

.. code:: bash

    $ roslaunch sr_robot_launch srhand.launch robot_description:=`rospack find sr_description`/robots/shadowhand_motor.urdf.xacro

-  The ``robot description`` param can be changed to start any of the
   available Shadow hands shown in the table.
-  If it is a left hand, ``hand_id:=lh`` should be added. For example:

   .. code:: bash

       $ roslaunch sr_robot_launch srhand.launch robot_description:=`rospack find sr_description`/robots/shadowhand_left_motor.urdf.xacro hand_id:=lh

-  Moveit will enable advanced behaviour (inverse kinematics, planning,
   collision detectection, etc...), but if it is not needed, you can set
   ``use_moveit:=false``

-  To start the dexterous hand plus
   (shadowhand\_motor\_plus.urdf.xacro), you can add the hand\_type like
   this:

   .. code:: bash

       $ roslaunch sr_robot_launch srhand.launch hand_type:=hand_e_plus


Shadow hand with UR10 arm
-------------------------

.. figure:: ../img/ur10hand.png
   :alt:

To start the simulation of the hand and the UR10 arm, you can run:

.. code:: bash

    $ roslaunch sr_robot_launch sr_right_ur10arm_hand.launch

or, for the left hand:

.. code:: bash

    $ roslaunch sr_robot_launch sr_left_ur10arm_hand.launch

Bimanual system
---------------

.. figure:: ../img/bimanual.png
   :alt:

To start the simulation of a bimanual system, you can run:

.. code:: bash

    $ roslaunch sr_robot_launch sr_bimanual.launch use_moveit:=true



.. |image0| image:: ../img/shadowhand_motor.png
.. |image1| image:: ../img/shadowhand_motor_biotac.png
.. |image2| image:: ../img/shadowhand_motor_ff_biotac.png
.. |image3| image:: ../img/shadowhand_motor_btsp.png
.. |image4| image:: ../img/shadowhand_motor_ellipsoid.png
.. |image5| image:: ../img/shadowhand_motor_th_ff_rf_ellipsoid.png
.. |image6| image:: ../img/shadowhand_muscle.png
.. |image7| image:: ../img/shadowhand_muscle_biotac.png
.. |image8| image:: ../img/shadowhand_lite.png
.. |image9| image:: ../img/shadowhand_extra_lite.png
.. |image10| image:: ../img/shadowhand_motor_plus.png
