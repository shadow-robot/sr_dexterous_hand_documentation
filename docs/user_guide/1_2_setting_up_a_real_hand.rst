-----------------------
Setting up a real hand
-----------------------

.. toctree::
   :maxdepth: 2
   :caption: Uni-manual system

   1_2_1_setting_up_a_real_hand_uni
   1_2_2_running_the_system_uni
   1_2_3_installing_the_software_uni
   1_2_4_saving_logs

Bimanual system
================

Connecting Cables
------------------

* Connect the hands to the NUC-CONTROL. It is very important that the exact USB-ethernet adapters are used.

  * The right hand should be connected to a USB->ethernet adapter labelled: ``HAND RIGHT``, which should be connected to one of the USB ports of the NUC-CONTROL (it does not matter which one).
  * The left hand should be connected to a USB->ethernet adapter labelled: ``HAND LEFT``, which should be connected to one of the USB ports of the NUC-CONTROL (it does not matter which one).

* Connect one USB->Ethernet adapter labelled ``NUC-CONTROL`` to another USB port on the NUC and the other USB->Ethernet labelled ``SERVER`` to any of the ports in your ``SERVER`` Laptop (provided by Shadow or a custom one).
* Connect the two adaptors together with an Ethernet cable.

You have been supplied with medium length Ethernet leads, but if you require a different length, you can simply use a standard commercial Ethernet Cat 5 cable, available from most computer parts suppliers. If you require internet connection in the laptop, connect an ethernet cable providing external internet connection to the back of the laptop, to an ethernet port labelled ``INTERNET``.

.. figure:: ../img/connecting_the_bimanual_hand.png
    :width: 100%
    :align: center
    :alt: Connections diagram

    Connections diagram

.. Source to edit the diagram: https://docs.google.com/drawings/d/1IOYFVruiCEKmIZpWwnUS8AJ-SWSNJJGQQxWrUoBa2Hk/edit?usp=sharing

* Finally, connect the external power supply to the hands using the metal Lemo connector, making sure to line up the red dots. If you require a longer or shorter cable, please contact us at support@shadowrobot.com.

Connection procedure
^^^^^^^^^^^^^^^^^^^^^
1. Connect the ethernet between the NUC and the laptop using the instructions above
2. Power on the laptop
3. Connect an ethernet cable providing external internet connection to the back of the laptop
4. Power on the NUC
5. Make sure the laptop has only 1 USB-Ethernet adapter connected to it.
6. In case of using another laptop than one provided, please follow the instructions below to install the software.
7. Power on the hand(s)
8. Connect the right hand to the USB-ethernet adapter labelled “HAND RIGHT” which should be plugged in to the NUC, as explained above
9. Connect the left hand to the USB-ethernet adapter labelled “HAND LEFT” which should be plugged in to the NUC, as explained above
10. Depending on what you want to launch: click on Launch Shadow Right Hand or Launch Shadow Left Hand or Launch Shadow Bimanual Hands. The hand(s) should vibrate and Rviz opens.
11. You can use the icons in “Shadow Demos” folder to close and open the hand(s) and run the standard demo(s), as well as save and upload ROS logs (send them to Shadow)

.. note::
    When you want to shut down the NUC, press and hold the power button of the NUC for at least 3 seconds and then let go.

Installing the software on a new PC
-----------------------------------

By default, we will provide machines that already have all the software set up for you.
However, even though each delivery will consist of a NUC-CONTROL machine for Hand's driver, the SERVER Laptop is optional.
In case you want to set up a custom machine as a client, please follow the instructions below.
The values for each field can be found in the Hand Delivery Instructions provided with the hand.

We have created a one-liner that is able to install Docker, download the docker image and create a new container for you.
It will also create desktop icons, one to start the container, one to launch the hand driver on the control box and one to save the log files locally.
To use it, you first need to have a PC with Ubuntu installed on it (preferably version 18.04), then follow these steps:

* **Get ROS Upload login credentials**

  If you want to upload technical logged data (ROS logs, backtraces, crash dumps etc.) to our server and notify the Shadow's software team to investigate your bug, then you need to enable logs uploading in the one-liner.
  In order to use this option you need to obtain a unique upload key. It can be found in the delivering instructions or by emailing sysadmin@shadowrobot.com. When you receive the key you can use it when running the one-liner installation tool.
  To enable the logs uploading you need to add the command line option ``--read-secure customer_key`` to the one-liner.
  After executing the one-liner, it will prompt you to enter your upload key and press enter to continue. Please copy and paste your key.

* **Run the one-liner**:

  The one-liner will install Docker, pull the image from Docker Hub, and create and run a container with the parameters specified. In order to use it, run the following command:

  1. Connect the ethernet between the NUC-CONTROL and the new PC using the instructions above
  2. Power on the new PC
  3. Connect an ethernet cable providing external internet connection to the back of the new PC
  4. Power on the NUC-CONTROL
  5. Make sure the new PC has only 1 USB-Ethernet adapter connected to it.
  6. Install the hand software on the new PC by running the following:
  7. Open a terminal in Ubuntu (Ctrl+Alt+T) and run:

  * ROS Melodic (Recommended):

    .. prompt:: bash $

       bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure customer_key product=hand_e ethercat_right_hand=<ethercat_right_hand> ethercat_left_hand=<ethercat_left_hand> config_branch=<config_branch> reinstall=true bimanual=true upgrade_check=true tag=melodic-release

  where ``<ethercat_right_hand>``, ``<ethercat_left_hand>`` and ``<config_branch>`` are values that will be provided in the Hand Delivery Instructions by Shadow.

  If you do not have an Nvidia graphics card, you can add nvidia_docker=false.

  Notice that you can set ``reinstall=false`` in case you do not want to reinstall the docker image and container.

  When it finishes it will show if it was successful or not and will create desktop icons on your desktop that you can double-click to launch the hand container, save the log files from the active containers to your desktop and perform various actions on the hand (open, close and demo).

  .. warning::
    If for whatever reason the installation does not proceed well or if it takes too long, contact us at support@shadowrobot.com with the error message. Also, try rerunning the installation script.
