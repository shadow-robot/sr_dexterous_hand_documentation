Installing the software on a new PC
-----------------------------------
By default, we will provide machines that already have all the software set up for you.
However, even though each delivery will consist of a NUC-CONTROL machine for Hand's driver, the SERVER Laptop is optional.
In case you want to set up a custom machine as a SERVER, please follow the instructions below.
The values for each field can be found in the Hand Delivery Instructions provided with the hand.

We have created a one-liner that is able to install Docker, download the docker image and create a new container for you.
It will also create desktop icons, one to start the container, one to launch the hand driver on the control box and one to save the log files locally.
To use it, you first need to have a PC with Ubuntu installed on it (preferably version 18.04), then follow these steps:

* **Get ROS Upload login credentials**

  If you want to upload technical logged data (ROS logs, backtraces, crash dumps etc.) to our server and notify the Shadow's software team to investigate your bug, then you need to enable logs uploading in the one-liner.
  In order to use this option you need to obtain a unique upload key. It can be found in the delivering instructions or by emailing sysadmin@shadowrobot.com. When you receive the key you can use it when running the one-liner installation tool.
  To enable the logs uploading you need to add the command line option ``--read-secure customer_key`` to the one-liner.
  After executing the one-liner, it will prompt you to enter your "Secure data input for customer_key". Please copy and paste here your key.

* **Run the one-liner**:

  The one-liner will install Docker, pull the image from Docker Hub, and create and run a container with the parameters specified. In order to use it, follow these instructions:

  1. Connect the ethernet between the NUC-CONTROL and the new PC using the instructions above
  2. Power on the new PC
  3. Connect an ethernet cable providing external internet connection to the back of the new PC
  4. Power on the NUC-CONTROL
  5. Install the hand software on the new PC by running the following on a terminal (Ctrl+Alt+T):

  ROS Melodic (Recommended) for a Right Hand:

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure customer_key ethercat_interface=<ethercat_interface> config_branch=<config_branch> product=<product> reinstall=true upgrade_check=true tag=melodic-release hand_side=right

  ROS Melodic (Recommended) for a Left Hand:

  .. prompt:: bash $

     bash <(curl -Ls bit.ly/run-aurora) server_and_nuc_deploy --read-secure customer_key ethercat_left_hand=<ethercat_interface> config_branch=<config_branch> product=<product> reinstall=true upgrade_check=true tag=melodic-release hand_side=left

  where ``<ethercat_interface>``, ``<config_branch>`` and ``<product>`` are values that will be provided in the Hand Delivery Instructions by Shadow.

  Product can be: hand_e, hand_lite or hand_extra_lite

  If you do not have an Nvidia graphics card, you can add nvidia_docker=false.

  You can also change ``reinstall=false`` in case you do not want to reinstall the docker image and container. When it finishes it will show if it was successful or not
  and will create desktop icons on your desktop that you can double-click to launch the hand container, save the log files from the active containers to your desktop and perform various actions on the hand (open, close and demo).
  
  More params and their explanation can be found `here. <https://github.com/shadow-robot/aurora/blob/master/ansible/inventory/server_and_nuc/group_vars/server.yml>`_


  .. warning::
    If for whatever reason the installation does not proceed well or it takes too long, contact us at support@shadowrobot.com with the error message. Also, try rerunning the installation script.

* **ROS MASTER and How to connect additional, non-Shadow computers with ROS to server laptop and NUC**

  The server laptop acts as the ROS MASTER and the NUC Docker container has export ROS_MASTER_URI=http://server:11311
  The ``server`` is ROS_MASTER_URI is resolved to the IP address of the server laptop due to being configured in /etc/hosts on the NUC host (which is shared with Docker container)

  To connect additional computers with ROS to server laptop and NUC ROS network (to control and see data from the hand), it is only necessary to have the server laptop and the additional non-Shadow computer with ROS on the same network.
  Connect the server laptop to an external wifi or use an ethernet cable connected to the server laptop's onboard ethernet port to connect the server laptop to the same network that the additional computer is in.

  In this additional network, let's say the server laptop acquires a new IP, e.g. 10.7.2.1

  Then, on the additional computer's /etc/hosts, include
  10.7.2.1 server

  Test it by pinging ``server`` from the additional computer. If it works, run the following in the terminal of the additional computer:

.. prompt:: bash $

     export ROS_MASTER_URI=http://server:11311

  Test if the additional computer can see the ROS topics and echo the contents:

  .. prompt:: bash $

     rostopic list

  .. prompt:: bash $

     rostopic echo /joint_states

  Now the additional computer is fully connected ROS MASTER of the server laptop.
  See the ``Software Description`` section and ``Command line interface`` subsection
