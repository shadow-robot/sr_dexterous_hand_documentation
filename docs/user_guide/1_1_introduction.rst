First time users
----------------
If you are unfamiliar with ROS and intend to use the ROS API, it is highly recommended that you read the `ROS Tutorials <http://www.ros.org/wiki/ROS/Tutorials>`_.

If you are unfamiliar with the terminal on Linux, you should look `here <https://askubuntu.com/questions/183775/how-do-i-open-a-terminal>`_.

Shadow software is deployed using Docker. Docker is a container framework where each container image is a lightweight, stand-alone, executable package that includes everything needed to run it. It is similar to a virtual machine but with much less overhead. Follow the instructions in the next section to get the latest Docker container of the hand driver and interface up and running.

Hardware specifications
-----------------------

Currently, we are providing a control machine together with the hand. However, if needed to be run on a custom machine, in order to run our software and the ROS software stack you will need to meet some hardware requirements.

+---------------+----------------------------------------------------------------------------------------------+
| CPU           | Intel i5 or above (i7 recommended)                                                           |
+---------------+----------------------------------------------------------------------------------------------+
| RAM           | 4GB or above (16 GB recommended)                                                             |
+---------------+----------------------------------------------------------------------------------------------+
| Hard Drive    | Fast HDD or SSD as laptop HDD are very slow (500 GB SSD recommended                          |
+---------------+----------------------------------------------------------------------------------------------+
| Graphics Card | Nvidia GPU (optional)                                                                        |
+---------------+----------------------------------------------------------------------------------------------+
| LAN           | A spare LAN port to connect the Hand (even with a USB to LAN adaptor)                        |
+---------------+----------------------------------------------------------------------------------------------+
| OS            | Ubuntu 18.04 Melodic (Active development), 16.04 Kinetic or 14.04 Indigo for older releases. |
+---------------+----------------------------------------------------------------------------------------------+

The most important one is to have a fast HDD or an SSD.


