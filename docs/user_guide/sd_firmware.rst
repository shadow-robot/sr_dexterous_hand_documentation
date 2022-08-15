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

Time frame
----------

The Palm firmware has a considerable amount of work to complete in the 1 millisecond time frame:

• Detect the incoming EtherCAT packet

• Download the command data from the ET1200

• Request sensor data from the motors

• Sample all of the joint sensors

• Request data from the tactile sensors

• Receive sensor data from the motors

• Transmit demand data to the motors

• Upload status data into the ET1200

In this diagram, we can see a breakdown of the time frame:








**SPI to ET1200:** All of the data must be written to the ET1200, before the next EtherCAT packet arrives. If it does not, then the packet's status data will be filled with zeros.

**SPI to Sensors:** The SPI bandwidth is really the limiting factor in the time frame. Data cannot be written back to the ET1200 until it has been collected by the MCU.

**CPU Busy:** We can see that the CPU is busy for most of the time, communicating with the ET1200, sampling sensors etc.

**CAN buses:** The CAN buses are close to maximum utilization. A little time is left during each frame to allow for re-transmission attempts. The time frame begins with a request-for-data message from the palm. The motors drivers respond immediately with their data. As soon as all 10 messages have been received, the palm sends out the demand values to all motor drivers.


Tactile sensors
---------------

The palm firmware supports different types of tactile sensor. The type of sensor is automatically detected, and the correct protocol is used between the hand and the sensor. The host PC is also informed of the sensor type so that it can interpret the data correctly. If more than one type of sensor is connected, then it is not possible to communicate with any of them, and no tactile sensor information will be available. The host will be informed of the conflict.

See 10.1 Distal Tactile Sensors for a list of supported tactile sensors.



Motor Firmware
--------------
