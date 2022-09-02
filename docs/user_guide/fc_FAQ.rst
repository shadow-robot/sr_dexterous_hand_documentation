Hand Frequently Asked Questions
=================================

A list of common issues and how to resolve them.

Hardware
---------

**Q: How do I know when the Hand is powered on?**

A: Lights will come on on the back of the hand and the fans will be audible.

**Q: The Hand will not power on, why is this?**

A: Check all connections from the power supply to the Hand. Check that the plug socket is turned on and the mains lead is plugged in. If all the connections are OK and the Hand still won't power on, contact us by sending an email to `Support <https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/full_manual/user_guide/sp_support.html>`_ 

**Q: The Hand is powered but I cannot connect to my PC.**

A: Check:
* Your ethernet cable connection
* The link light on your computer’s ethernet port is in a fixed state, not flashing
* You’re using the right interface - instructions to check this are `here <https://shadow-robot-company-dexterous-hand-and-arm.readthedocs-hosted.com/en/latest/user_guide/1_4_Installing_the_software.html#check-your-hand-interface-id>`_

**Q: Can the Hand be controlled wirelessly?**

A: The hand is currently not wireless, as it needs both power and data cables attached. 

**Q: How do the tactile sensors work and how do they communicate?**

A: There are two types of tactile sensing for the fingertips:
* **PST**: these are simple sensors, fitted as standard, which measure the air pressure within a bubble at the fingertip. When the fingertips press an object, the sensor detects the change in pressure. The sensor incorporates an automatic drift and temperature compensation algorithm (essentially a high pass filter with an extremely low cut off frequency).
* **Biotacs**: please refer to their website for more information: https://syntouchinc.com/

Both tactile sensors are attached by a cable to the Finger Proximal Boards.

**Q: Can I fix the Hand myself?**

A: We provide a set of tools that allow you to re-tension the tendons and complete some other tasks on the Hand yourself, so it won’t have to be sent back to Shadow. Where possible, one of our support staff will organise a video call with you to talk you through how to fix any problems you may have. For more complex diagnostic issues, the Hand will need to be sent back to Shadow.

Software
-----------

**Q: The Hand doesn’t react after startup of the container**

A: Check that:
* The interface ID is set correctly
* You have set the correct branch for sr_config
If it is still not responding, power cycle the Hand and try again.

**Q: Why is the control loop now running on the NUC? Can I run the system without the NUC if I prefer?**

A: The NUC was introduced to avoid cycle skips for certain setups. This is more common in our more complex systems when we are running other heavy programs on the same laptop. If you are running just the hand locally you may not experience this issue, but you will start to see overruns if running other computationally expensive software. We strongly advise that the Hand is run using the NUC to ensure best performance.

However, using the NUC is not mandatory and no changes have been made to the firmware. We have provided a set of icons that are in a Desktop folder called “Shadow Advanced Launchers”. One of these icons, called “Launch Local Shadow Hand”, will allow you to launch and run the hand connected locally to your laptop. More information on the Advanced Launch icons can be found `here <https://dexterous-hand.readthedocs.io/en/master/user_guide/1_2_10_icons_for_hand.html#shadow-advanced-launchers>`_

**Q: How does the NUC fit in with secondary development for the Hand?**

A: The NUC only handles low level communication with the Hand as it is only running the driver. If you would like to do your own development with the Hand, continue doing so on the laptop and your scripts will be able to control the Hand. If you would like to change something on the NUC side, the icon called “Launch NUC Container” (within the Shadow Advanced Launchers desktop folder) will give you quick access to the NUC and provide a terminal with full access to the driver. 

**Q: Why do you use Docker?**

A: Using Docker helps us with customer support and development as we are able to replicate our customers’ exact software environment if problems arise. If you would like to start the container within Docker without starting the whole system, this can be done using the “1 - Launch Server Container” and “Launch NUC Container” icons within the Shadow Advanced Launchers desktop folder.  More information on the Advanced Launch icons can be found `here <https://dexterous-hand.readthedocs.io/en/master/user_guide/1_2_10_icons_for_hand.html#shadow-advanced-launchers>`_

**Q: Can I install Shadow Software using my own method?**

A: We officially only offer support for installing the software using Docker and our Aurora script to avoid migration issues and other difficulties. The rosinstall file that we use to create the docker image can be found `here <https://github.com/shadow-robot/sr-build-tools/blob/master/data/shadow_robot-melodic.rosinstall>`_

If you have any questions about the installation one-liner, please contact us by sending an email to `Support <https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/full_manual/user_guide/sp_support.html>`_ 

**Q: Sometimes when I plan a trajectory using MoveIt! in Rviz, the Hand doesn't move.**

A: This is often due to the sensors getting confused about their start state. Simply try trajectory planning again, or power cycle the Hand.

**Q: I am having trouble connecting to the NUC, or am receiving errors to do with the DHCP server.**

A: This is a known error that has been resolved in our latest software releases. In order to integrate these changes, please run the latest Aurora command following your Delivery Instructions. If you have any questions, please contact us at `Support <https://shadow-robot-company-dexterous-hand.readthedocs-hosted.com/en/full_manual/user_guide/sp_support.html>`_ 

Common Error Messages
^^^^^^^^^^^^^^^^^^^^^^

**Error: EC slave 1 not in init state**

Power cycle the Hand. This error will not affect the performance of the system.

**WARNING: disk usage in log directory [...] is over 1GB.**

A: This is just explaining that the logs are taking up a lot of disk space. Use the ‘rosclean’ command to clear these if you like.


Other Questions
---------------

**Q: I would like to organise a second training session for some of my team that weren’t able to make it to the first one. Is this possible?**

A: We will likely be able to give you a second introduction to the system or organise a meeting to answer any further questions you may have. Please contact alex@shadowrobot.com to organise a time that works for you. 

**Q: I have bought multiple Hands from Shadow in the past, but the orders have included different servers and Ubuntu distributions. Why is this?**

A: We sometimes change hardware suppliers if they are not meeting our lead time or spec requirements. We ensure that all of the servers and NUCs we supply are of high enough spec to work well with our software. We update the Ubuntu and ROS distributions we use to make use of the most up to date software available to us, and maintain compatibility.
