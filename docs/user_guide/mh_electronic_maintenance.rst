Electronic Maintenance
=======================

Strain Gauge failure
--------------------

If one strain gauge appears to have failed, it is possible that the connector has come loose. Swing out the motor, and examine the connectors. Re-insert with a small pair of tweezers. If this was not the cause of the problem, please contact Shadow Robot for advice sending an email to support@shadowrobot.com.

Position sensor failure
-----------------------

If a position sensor appears to be returning incorrect values, or a constant value, first check that the raw sensor data is working:
rostopic echo /debug_etherCAT_data/sensor[?]
You can also check the state of the raw sensor from the Shadow Hand Calibration plugin in the GUI.
