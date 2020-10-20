Running the system
===================

Understanding the icons on the SERVER Laptop
--------------------------------------------

The icons look like this:

* Desktop icons:

  .. figure:: ../img/icons.png
      :align: center
      :alt: Desktop icons

  * Launch Shadow Right (or Left) Hand - launches the right (or left) hand
  * Shadow ROS Logs Saver - used to save the hand logs and upload them to AWS
  * Shadow NUC RQT - opens RQT window running within the NUC machine, allows access to ROS plugins

* Within the ``Shadow Demos`` folder you will find following icons (use only when driver is running):

  .. figure:: ../img/shadow_demos.png
      :align: center
      :alt: Desktop icons

  * Close Right (or Left) Hand - moves hand into pack position
  * Demo Right (or Left) Hand - starts a program running several hand demos
  * Open Right (or Left) Hand - moves hand into fully open position

* Within the ``Shadow Advanced Launchers`` folder you will find following icons:

  .. figure:: ../img/shadow_advanced_launchers.png
      :align: center
      :alt: Desktop icons

  * The following icons run in succession are the equivalent of using the desktop icon ``Launch Shadow Hand``:
    * Launch Server Container - starts docker container on the server machine only
    * Launch Server ROSCORE - only starts roscore on the server side
    * Launch NUC Container and Hardware Control Loop - starts the hand driver only, on the NUC side
    * Launch Server GUI - Start GUI on the server side allowing user to control movements of the hand

  * The above four icons run in succession are the equivalent of using the ``Launch Shadow Hand`` icon.

    * Launch Local Shadow Hand - icon to start the hand when it is plugged directly in to the server machine
    * Launch NUC container - start docker container on the NUC without starting the driver


Starting the driver
-------------------

* **Shadow Hand Driver**
  Launch the driver for the Shadow Hand using the desktop icon 'Launch Hand'.
  If you want to launch the hand locally (not recommended), plug in the hand ethernet adapter to the laptop and use the Advanced Launch Icon - ``Launch Local Shadow Hand``.

* **Lights in the hand**:
  When the ROS driver is running you should see the following lights on the Palm:

  ========================   =============       ================    =================================
  Light                      Colour              Activity            Meaning
  ========================   =============       ================    =================================
  Run                        Green               On                  Hand is in Operational state
  CAN1/2 Transmit            Blue                V.fast flicker      Demand values are being sent to the motors
  CAN1/2 Receive             Blue                V.fast flicker      Motors are sending sensor data
  Joint sensor chip select   Yellow              On                  Sensors being sampled
  ========================   =============       ================    =================================

  After killing the driver, the lights will be in a new state:

  ========================   =============       ================    =================================
  Light                      Colour              Activity            Meaning
  ========================   =============       ================    =================================
  Run                        Green               Blinking            Hand is in Pre-Operational state
  CAN1/2 Transmit            Blue                Off                 No messages transmitted on CAN 1/2
  CAN1/2 Receive             Blue                Off                 No messages received on CAN 1/2
  Joint sensor chip select   Yellow              Off                 Sensors not being sampled
  ========================   =============       ================    =================================