Installing the software on a new PC
====================================

.. note:: By default, we will provide machines that already have all the software set up for you.
However, even though each delivery will consist of a NUC-CONTROL machine for Hand's driver (which we always recommend to use), the SERVER Laptop is optional.
In case you want to set up a custom machine as a SERVER, please follow the instructions below.
The values for each field can be found in the ``Hand Delivery Instructions`` provided with the hand.

Hardware specifications
-----------------------

In order to run our software and its dependencies you will need to meet some hardware requirements.

+---------------+------------------------------------------------------------------------------------------------------------+
| CPU           | Intel i5 or above (i7 recommended)                                                                         |
+---------------+------------------------------------------------------------------------------------------------------------+
| RAM           | 4GB or above (16 GB recommended)                                                                           |
+---------------+------------------------------------------------------------------------------------------------------------+
| Hard Drive    | Fast HDD or SSD as laptop HDD are very slow (500 GB SSD recommended)                                       |
+---------------+------------------------------------------------------------------------------------------------------------+
| Graphics Card | Nvidia GPU (optional)                                                                                      |
+---------------+------------------------------------------------------------------------------------------------------------+
| LAN           | A spare LAN port to connect the Hand (even with a USB to LAN adaptor)                                      |
+---------------+------------------------------------------------------------------------------------------------------------+
| OS            | Ubuntu 20.04 Noetic (Active development), 18.04 Melodic, 16.04 Kinetic or 14.04 Indigo for older releases. |
+---------------+------------------------------------------------------------------------------------------------------------+

The most important one is to have a fast HDD or an SSD.

We have created a one-liner that is able to install Docker, download the Docker image and create a new container for you.
It will also create desktop icons, one to start the container, one to launch the hand driver on the control box and one to save the log files locally.
To use it, you first need to have a PC with Ubuntu installed on it, then follow the next steps.

Get ROS Upload login credentials
---------------------------------

If you want to upload technical logged data (ROS logs, backtraces, crash dumps etc.) to our server and notify the Shadow's software team to investigate your bug, then you need to enable logs uploading in the one-liner.
In order to use this option you need to obtain a unique upload key. It can be found in the delivering instructions or by emailing sysadmin@shadowrobot.com. When you receive the key you can use it when running the one-liner installation tool.
To enable the logs uploading you need to add the command line option ``--read-secure customer_key`` to the one-liner.
After executing the one-liner, it will prompt you to enter your "Secure data input for customer_key". Please copy and paste here your key.

Run the one-liner
------------------

The one-liner will install Docker, pull the image from Docker Hub, and create and run a container with the parameters specified. In order to use it, follow these instructions:

  1. Connect the ethernet between the NUC-CONTROL and the new PC using the instructions above
  2. Power on the new PC
  3. Connect an ethernet cable providing external internet connection to the back of the new PC
  4. Power on the NUC-CONTROL
  5. Install the hand software on the new PC by running the following on a terminal (Ctrl+Alt+T):

ROS Melodic (Recommended) for a Right Hand:

.. prompt:: bash $

   bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure customer_key ethercat_right_hand=<ethercat_interface> config_branch=<config_branch> product=<product> reinstall=true upgrade_check=true tag=melodic-release hand_side=right

ROS Melodic (Recommended) for a Left Hand:

.. prompt:: bash $

   bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure customer_key ethercat_left_hand=<ethercat_interface> config_branch=<config_branch> product=<product> reinstall=true upgrade_check=true tag=melodic-release hand_side=left
     
ROS Melodic (Recommended) for a Bimanual Hand System:

.. prompt:: bash $

   bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure customer_key product=hand_e ethercat_right_hand=<ethercat_right_hand> ethercat_left_hand=<ethercat_left_hand> config_branch=<config_branch> reinstall=true upgrade_check=true tag=melodic-release bimanual=true

where ``<ethercat_interface>``, ``<config_branch>`` and ``<product>`` are values that will be provided in the Hand Delivery Instructions by Shadow.

Product can be: hand_e, hand_lite or hand_extra_lite

If you do not have an Nvidia graphics card, you can add nvidia_docker=false.

You can also change ``reinstall=false`` in case you do not want to reinstall the Docker image and container. When it finishes it will show if it was successful or not and will create desktop icons on your desktop that you can double-click to launch the hand container, save the log files from the active containers to your desktop and perform various actions on the hand (open, close and demo).
  
More params and their explanation can be found `here. <https://github.com/shadow-robot/aurora/blob/master/ansible/inventory/server_and_nuc/group_vars/server.yml>`_


.. warning::
   If for whatever reason the installation does not proceed well or it takes too long, contact us at support@shadowrobot.com with the error message. Also, try rerunning the installation script.
