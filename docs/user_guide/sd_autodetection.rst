Hand autodetection 
--------------------

This feature (**new in Noetic**) allows users to detect Shadow Hands without knowing the ethernet interface or the hand serial and run launchfiles without needing to provide detailed information about the hands. It is implemented in the `sr_hand_detector package <https://github.com/shadow-robot/sr_hand_detector>`_ and consists of two scripts.

Installation
^^^^^^^^^^^^^

In all Shadow's docker images the feature will be available out of the box, however, for custom setups, you might need to install it manually. Recommended way is just to use debian installation:

.. code-block:: bash

   sudo apt update && sudo apt install ros-<rosdistro>-sr-hand-detector

If for some reason a manual installation is required, you can follow steps below:

1. Clone the repository to your ROS workspace
2. Compile the code
3. Copy both executables of the sr_hand_detector package (found in ``<your_workspace>/devel/lib/sr_hand_detector``) to ``/usr/local/bin``.
4. Give one of the executables capability to access ethernet devices:

.. code-block:: bash

   sudo setcap cap_net_raw+ep sr_hand_detector_node

Finally, if you want to use the autodetection feature with our launchfiles, you need to clone `sr_hand_config package <https://github.com/shadow-robot/sr_hand_config>`_ into your workspace.

sr_hand_detector_node
^^^^^^^^^^^^^^^^^^^^^^
The script is purely for hand detection. Usage: 

.. code-block:: bash

   sr_hand_detector_node

Example output:

.. code-block:: bash

   Detected hand on port: enx000ec653b31a
   Hand's serial number: 634

Apart from the console output, all detected hand ethernet port names together with corresponding hand serial numbers will be set inside of the /tmp/sr_hand_detector.yaml file.

If there are no hands detected on any of the ports, a warning will be shown:

.. code-block:: bash

   No hand detected on any of the ports!

sr_hand_autodetect
^^^^^^^^^^^^^^^^^^^

This script is a launchfile wrapper, and allows users to run Shadow Robot launch files without providing information like hand serial, ethercat port or hand side. Example usage:

.. code-block:: bash

   sr_hand_autodetect roslaunch sr_robot_launch srhand.launch sim:=false

which will effectively run:

.. code-block:: bash

   roslaunch sr_robot_launch srhand.launch sim:=false eth_port:=<eth_port> hand_serial:=<hand_serial> side:=<hand_side> hand_type:=<hand_type> mapping_path:=<mapping_path>

When using the wrapper, all the necessary information is extracted from the `sr_hand_config package <https://github.com/shadow-robot/sr_hand_config>`_.
