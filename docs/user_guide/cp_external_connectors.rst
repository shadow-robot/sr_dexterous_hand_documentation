External Connectors
====================

Power
-----







Peripheral Connectors
---------------------

The following connectors are available for connecting external peripherals.








**Auxiliary Analog:** This connector allows you to add up to four extra analog sensors to the
hand. These sensor channels are always sampled at 1000Hz, and are available at the host.
They are currently published in /debug_etherCAT_data/sensors[26 .. 28]. To assemble a
connector for this socket, use the following part numbers:


Harwin M30F1100600, Harwin M30-1010046.


1. 5v regulated supply


2. GND regulated supply


3. Analog input channel 0


4. Analog input channel 1


5. Analog input channel 2


6. Analog input channel 3


**Auxiliary SPI:** An external SPI device may be connected here, e.g. an ADC, DAC, or I/O
expander. The palm may be able to auto-detect the type of device connected, and inform the
host. Currently, the palm supports only three devices:


• MCP3208       -       8 channel, 12-bit ADC
• MCP3204       -       4 channel, 12-bit ADC
• MCP3202       -       2 channel, 12-bit ADC




The auto-detection of these devices is not completely reliable, since the MCP320x chips have
no explicit autodetection mechanism. The palm attempts to read all eight channels. If it reads
values other than 0x000 or 0xFFF, it assumes an ADC exists. Therefore, if the analogue sensors
are giving exactly 0v or 5v on every channel, the palm may fail to autodetect.


If the palm fails to autodetect a device, it assumes the device is an MCP3208.


To assemble a connector for this socket, use the following part numbers: Harwin M30F1100600,
Harwin M30-1010046.


1. 6v supply


2. GND supply


3. Chip Select


4. Clock


5. Master out, Slave In


6. Slave Out, Master In


6v supply


**ICD3 Socket:** This allows you to re-program new firmware into the Palm's PIC32 MCU. See the
section on re-programming the palm. You have been supplied with an adaptor to connect the
ICD3 programmer here.
