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


Status data for motor hand
--------------------------

Status data is sent from the palm, and received by the host. It consists of the following:

+----------------------------------+---------------+-------------------------------------------------+
| Item                             | Size          | Description                                     |
+==================================+===============+=================================================+
| EDC_Command                      | 32 bits       | Copy of the same value from command data        |
+----------------------------------+---------------+-------------------------------------------------+
| Joint sensor/IMU data            | 16 bits x 37  | All of the joint sensors, the                   |
|                                  |               | Auxiliary Analog channels, and the IMU sensors. |
+----------------------------------+---------------+-------------------------------------------------+
| Motor data type                  | 32 bits       | Copy of the same value from command data        |
+----------------------------------+---------------+-------------------------------------------------+
| Even or Odd motors ?             | 16 bits       | Copy of the same value from command data        |
+----------------------------------+---------------+-------------------------------------------------+
| Which motor data arrived         | 32 bits       | Flags indicate which CAN messages were seen     |
+----------------------------------+---------------+-------------------------------------------------+
| Which motor data had errors      | 32 bits       | Flags indicate that the wrong type of data was  |
|                                  |               | sent by this motor.                             |
+----------------------------------+---------------+-------------------------------------------------+
| Motor data                       | 16 bits x2 x10| Torque + one other sensor from 10 motors.       |     
+----------------------------------+---------------+-------------------------------------------------+
| Tactile sensor data type         | 32 bits       | Copy of the same value from command data        |
+----------------------------------+---------------+-------------------------------------------------+      
| Which tactile data is valid ?    | 16 bits       | Flags indicate which tactile data is valid.     |
+----------------------------------+---------------+-------------------------------------------------+
| Tactile sensor data              |  16 bits x8 x5|                                                 |
+----------------------------------+---------------+-------------------------------------------------+



Motor Firmware
--------------
