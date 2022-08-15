Firmware
==========

Palm firmware
--------------

The palm firmware is responsible for the following:

• Managing the ET1200 EtherCAT ASIC and EtherCAT state

• Receiving command data from the ET1200

• Transmitting the contents of the command packet to the motors

• Receiving data from the motors

• Sampling data from the joint sensors

• Sampling data from the Tactile sensors

• Loading status data into the ET1200

Command data for motor hand
---------------------------

Command data is sent from the host, and received by the palm. It consists of the following:

+----------------------------------+---------------+-------------------------------------------------+
| Item                             | Size          | Description                                     |
+==================================+===============+=================================================+
| EDC_Command                      | 32 bits       | Used for switching the palm into test mode      |
+----------------------------------+---------------+-------------------------------------------------+
| Motor data type request          | 32 bits       | Which sensor data should the motors return?     |
+----------------------------------+---------------+-------------------------------------------------+
| Even or Odd motors?              | 16 bits       | Which motors should return data?                |
+----------------------------------+---------------+-------------------------------------------------+
| Type of motor demand             | 32 bits       | Are we demanding torque or PWM? Can also        |
|                                  |               | be used to send config values to the motors.    |
+----------------------------------+---------------+-------------------------------------------------+
| Motor demand data                | 16 bits x 20  | All 20 torque or PWM demands.                   |
|                                  |               | May also contain config data for the motors.    |
+----------------------------------+---------------+-------------------------------------------------+
| Tactile sensor data type request | 32 bits       | Which type of sensor data should                |
|                                  |               | the tactile sensors return?                     |
+----------------------------------+---------------+-------------------------------------------------+




Motor Firmware
--------------
