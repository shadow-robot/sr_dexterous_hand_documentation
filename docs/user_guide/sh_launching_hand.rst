Launching the hands
===================

Depending on what you want to launch: click on ``Launch Shadow Right Hand`` or ``Launch Shadow Left Hand`` or ``Launch Shadow Bimanual Hands``. The hand(s) should vibrate and Rviz opens.

.. warning:: If you want to launch the hand on the SERVER laptop without using the NUC-CONTROL (not recommended), plug in the hand Ethernet adapter to the laptop and use the Shadow Advanced Launchers folder icon - ``Launch Local Shadow Right Hand``, ``Launch Local Shadow Left Hand`` or ``Launch Local Shadow Bimanual Hands``.

You can use the icons in “Shadow Demos” folder to close and open the hand(s) and run the standard demo(s), as well as save and upload ROS logs.

When shutting down the hand(s) after use, please make sure to use the *Shadow Close Everything* icon as this closes the software gracefully and also does a final calibration jiggle to ensure the strain gauges are zeroed.

Jiggling
---------

On reset, all of the strain gauges (torque sensors) in the motors need to be zeroed. This happens automatically. The motors are driven back and forth to try to relieve any tension on the tendons. Then both gauges are zeroed. You will therefore see all joints of the hand move slightly on power up or reset.

Power off
----------
Power off the hand by disconnecting the power supply. When you want to shut down the NUC, press and hold the power button of the NUC for at least 3 seconds and then let go. Power off the Server with the power off button available in Ubuntu.
