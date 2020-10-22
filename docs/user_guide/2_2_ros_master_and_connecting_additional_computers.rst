ROS MASTER and connecting additional computers
==============================================

The server laptop acts as the ROS MASTER and the NUC Docker container has export ROS_MASTER_URI=http://server:11311
The ``server`` is ROS_MASTER_URI is resolved to the IP address of the server laptop due to being configured in /etc/hosts on the NUC host (which is shared with Docker container)

To connect additional computers with ROS to server laptop and NUC ROS network (to control and see data from the hand/arm), it is only necessary to have the server laptop and the additional non-Shadow computer with ROS on the same network.
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
