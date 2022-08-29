Saving States
--------------
To save a state you must first be connected to the warehouse. After launching the hand, click the green **Connect** button in the 'Context' tab of rviz.

.. image:: ../img/sd_rviz_warehouse_connect.png

If you have connected successfully you should see two new buttons, **Reset database** and **Disconnect**, as can be seen in the following picture:

.. image:: ../img/sd_rviz_warehouse_connected.png

Next, go to the 'Stored States' tab in 'Motion Planning'. Here you have full control over the saved states in the warehouse. You can then follow these steps:

* move the hand to the grasp position
* Go to the 'Planning' tab and in the 'Select Goal State' select 'current' and click **update**.

.. image:: ../img/sd_rviz_select_goal_state.png

* Finally, go to the 'Stored States' tab and click the button **Save Goal** under the 'Current State' group. A prompt will appear to ask you to name the state. Once named, you can plan to and from this state.

.. image:: ../img/sd_save_state.png
