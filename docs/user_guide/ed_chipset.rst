Chipset
=======


PIC32: This is a 32-bit MIPS CPU which performs all of the sensor sampling, and handles the
flow of all the data in the hand.
ET1200: This is the EtherCAT ASIC, dealing with the reception and transmission of EtherCAT
packets to and from the PC.
MCP3208: These are 12-bit ADCs used throughout the hand. There are two in the palm, and
one in each finger. There are two on the underside of the Palm PCB, and one in each finger.
A1321: These are Hall effect sensors used to measure the position of all joints of the hand.
EN5311QI: This is an ultra high efficiency switching regulator which powers the PIC32 and the
ET1200.
MPU-6000: (Optional) This is a 6-axis accelerometer / gyroscope which may be useful to some
customers.
