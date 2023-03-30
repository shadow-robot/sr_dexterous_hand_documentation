Environment
=================

Robot Operating System (ROS), Linux and Docker
----------------------------------------------

Our systems work within the ROS framework. 

"ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an 
operating system, including hardware abstraction, low-level device control, implementation of commonly-used 
functionality, message-passing between processes, and package management. It also provides tools and libraries for 
obtaining, building, writing, and running code across multiple computers." - ROS.org

If you are unfamiliar with ROS and intend to use the ROS API, you can find the fundamental ROS concepts explained `here <http://wiki.ros.org/ROS/Concepts>`_ and a technical overview of the implementation of ROS `here <http://wiki.ros.org/ROS/Technical%20Overview>`_.
It is highly recommended that you also check the `ROS Tutorials <http://www.ros.org/wiki/ROS/Tutorials>`_.

If you are unfamiliar with Linux and its terminal on Linux, you should look `here <https://ubuntu.com/tutorials/command-line-for-beginners#1-overview>`_.

Shadow software is deployed using Docker. Docker is a container framework where each container image is a lightweight, stand-alone, executable package that includes everything needed to run it. It is similar to a virtual machine but with much less overhead. You can find more information `here <https://www.docker.com/resources/what-container/>`_.

Repositories
------------

Our code is split into different ROS repositories:

* `sr_common <https://github.com/shadow-robot/sr_common>`_: This repository contains the bare minimum for communicating with the Shadow Hand from a remote computer (urdf models and messages).
* `sr_core <https://github.com/shadow-robot/sr_core>`_: These are the core packages for the Shadow Robot hardware and simulation.
* `sr_interface <https://github.com/shadow-robot/sr_interface>`_: This repository contains the high level interface and its dependencies for interacting simply with our robots.
* `sr_tools <https://github.com/shadow-robot/sr_tools>`_: This repository contains more advanced tools that might be needed in specific use cases.
* `sr_visualization <https://github.com/shadow-robot/sr_visualization>`_: This repository contains the various rqt_gui plugins we developed.
* `sr_hand_config <https://github.com/shadow-robot/sr_hand_config>`_: This repository contains the customer specific configuration for the Shadow Robot Hand.
