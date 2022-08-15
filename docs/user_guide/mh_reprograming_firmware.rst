Reprograming Firmware
======================


**NOTE: Any change in the robot firmware may cause unpredictable results and damage the hardware. Please, confirm with Shadow before making any change in the robot firmware.**

Motors
------

New motor firmwares can be bootloaded using a plugin in the GUI. You can find more information in the link in section 4.

Palm
----

There is currently no way to reprogram the palm MCU from the PC. The only way to do this is to connect a Microchip ICD3 to the palm PCB via the supplied gold pin connector.










Locate the supplied gold-pin programming adaptor, and connect it to your ICD3. Remove the four screws holding the palm's polycarbonate cover, and remove the cover. Open MPLAB, and open the EDC_Palm.mcw workspace.
Import the firmware. File → Import. Locate the .HEX file.
Connect to the ICD2. Programmer → Select Programmer → MPLAB ICD3
Wait for MPLAB to connect to the ICD3.







Push the gold-pin connector into the holes in the palm PCB as shown above. MPLAB should say “Target detected” in the Output window.









Click Program.










Wait until MPLAB says “Programming/Verify complete”.








Remove the gold-pin connector. You may need to press the reset button or power-cycle the hand for it to re-boot correctly. You will also need to re-start the ROS driver if it is currently running.
