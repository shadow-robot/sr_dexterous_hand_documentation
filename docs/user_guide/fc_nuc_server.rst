Why do we use a NUC?
==========================

We have migrated the control loop to a separate NUC computer.
The reason for that is that we have experienced high cycle overruns with certain setups.
We have seen the issue especially in complex systems such as Teleoperation when we are running other heavy programs on the same laptop.
If you run just the hand, it might be ok but if you run arm + hand + any other computationally expensive software such as vision, etc. you will start to see the overruns.

Testing the overuns
--------------------
You can follow these instructions to test the overruns in your computer:

1. Install stress on the computer where the hand is running:

   .. prompt:: bash $

      sudo apt install stress

2. Start the hand driver (using either the ``Launch Shadow Right/Left/Bimanual Hand(s)`` icon if the hand is connected to the NUC or ``Launch Local Shadow Right/Left/Bimanual Hand(s)`` (in Shadow Advanced Launchers folder)
3. Start this command on the computer host where the hand is running:

   .. prompt:: bash $

      stress --cpu 8 --io 4 --vm 2 --timeout 200s --vm-bytes 128M

4. In the Docker container of the computer where the hand is running (either Server container if hand running on NUC or if hand is running on the laptop, right-click on the Terminator window and “Split Horizontally”, run this (rh is right hand, if using a left hand use lh)

   .. prompt:: bash $

      rosrun sr_utilities_common overrun_experiments.py -ht hand_e -t 120 -id rh

5. Wait 2 minutes. You should see this:

   .. prompt:: bash $

      Your data has been recorded to ./overruns_data.txt file.
      Overrun average: <overrun_average> Drop average: <drop_average>

We normally want overrun_average to be less than 0.05 and the drop_average (dropped packets)  to be less than 0.1.
This would mean that over 120 seconds, there should be less than 6 overruns and less than 12 drops.
You can also open the overruns_data.txt file to see what the overruns and drops have been.

What will happen if a NUC is not used?
----------------------------------------

If we don’t use a NUC, and even if we use a moderately powerful laptop (i7-8750H/9750H/10750H and 16GB RAM and NVIDIA 1650/1650 TI GPU),
when the laptop is stressed (e.g. running any software on the laptop or opening a browser), we get an overrun average of about 11 and a drop average
of about 0.45, which means that over 120 seconds we would have 1320 overruns and 54 drops.
This would seriously degrade the real-time performance of the hand.

Running the hand without the NUC
---------------------------------

Running the hand using the NUC is recommended but not mandatory.
There are extra icons to start the hand without a NUC in the "Shadow Advanced Launchers" folder.
In that folder, you can use the icon ``Launch Local Shadow Right/Left/Bimanual Hand(s)`` to run the hand without a NUC (hand has to be connected to server laptop using the same USB-Ethernet adapter).
Running the hand with an arm is not supported without a NUC. 
More information on the Advanced Launch icons can be found `here <https://dexterous-hand.readthedocs.io/en/master/user_guide/1_2_10_icons_for_hand.html#shadow-advanced-launchers>`_.

