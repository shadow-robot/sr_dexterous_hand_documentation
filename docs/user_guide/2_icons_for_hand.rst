Desktop icons for the hand
==========================

The following icons will be available on the server laptop desktop for launching and controlling the right, left and bi-manual hands. Please note that in Ubuntu 20.04 these icons will be located inside a folder called 'Shadow Launcher' on the desktop. You might need to right Click with your mouse and select 'Allow Launching'.

Main desktop icons
-------------------

.. table::
   :class: tight-table
   
   +--------------------------------------------------+--------------------------------------------------+
   | Icon text                                        |  Icon explanation                                | 
   +==================================================+==================================================+
   | Shadow ROS Logs Saver and Uploader               | Saves ROS logs from server and NUC, uploads them |
   |                                                  | to Shadow servers and emails them to Shadow      |
   +--------------------------------------------------+--------------------------------------------------+
   | Launch Shadow Right Hand                         | Launches the right hand (container, ROS core,    |
   |                                                  | NUC hardware control loop, server GUI)           |
   +--------------------------------------------------+--------------------------------------------------+
   | Launch Shadow Left Hand                          | Launches the left hand (container, ROS core,     |
   |                                                  | NUC hardware control loop, server GUI)           |
   +--------------------------------------------------+--------------------------------------------------+
   | Launch Shadow Bimanual Hands                     | Launches the both hands (container, ROS core,    |
   |                                                  | NUC hardware control loop, server GUI)           |
   +--------------------------------------------------+--------------------------------------------------+   
   | Shadow NUC RQT                                   | Once the hand has been launched, this icon       |
   |                                                  | starts ROS RQT inside the NUC's docker container |
   +--------------------------------------------------+--------------------------------------------------+
   | Dexterous Hand Documentation                     | Opens online documentation if internet connected |
   |                                                  | or offline documentation if no internet          |
   +--------------------------------------------------+--------------------------------------------------+
   | Shadow Close Everything                          | Cleanly stops ROS processes, closes containers,  |
   |                                                  | and closes all Shadow related terminals          |
   +--------------------------------------------------+--------------------------------------------------+

Shadow Demos folder
--------------------

The following icons will be available in the Shadow Demos folder. They will only work once the hand has been launched.

.. table::
   :class: tight-table
   
   +--------------------------------------------------+--------------------------------------------------------------+
   | Icon text                                        |  Icon explanation                                            | 
   +==================================================+==============================================================+
   | Close Left Hand                                  | Once the hand has been launched, this icon will              |
   |                                                  | close (pack) the left hand                                   | 
   +--------------------------------------------------+--------------------------------------------------------------+
   | Close Right Hand                                 | Once the hand has been launched, this icon will              |
   |                                                  | close (pack) the right hand                                  |
   +--------------------------------------------------+--------------------------------------------------------------+
   | Close Bimanual Hands                             | Once bimanual hands have been launched, this icon will       |
   |                                                  | close (pack) both hands                                      |
   +--------------------------------------------------+--------------------------------------------------------------+
   | Biotac Demo Left Hand/Demo Left Hand             | Once the hand has been launched, this icon will              |
   |                                                  | run various (tactile/keyboard-actived) left hand demos       |
   +--------------------------------------------------+--------------------------------------------------------------+
   | Biotac Demo Right Hand/Demo Right Hand           | Once the hand has been launched, this icon will              |
   |                                                  | run various (tactile/keyboard-actived) right hand demos      |
   +--------------------------------------------------+--------------------------------------------------------------+
   | Biotac Demo Bimanual Hands/Demo Bimanual Hands   | Once bimanual hands have been launched, this icon will       |
   |                                                  | run various (tactile/keyboard-actived) bimanual hands demos  |
   +--------------------------------------------------+--------------------------------------------------------------+
   |       | Open Left Hand                           | Once the hand has been launched, this icon will              |
   |                                                  | fully open the left hand                                     |
   +--------------------------------------------------+--------------------------------------------------------------+
   |       | Open Right Hand                          | Once the hand has been launched, this icon will              |
   |                                                  | fully open the right hand                                    |
   +--------------------------------------------------+--------------------------------------------------------------+
   |   | Open Bimanual Hands                          | Once bimanual hands have been launched, this icon will       |
   |                                                  | fully open both hands                                        |
   +--------------------------------------------------+--------------------------------------------------------------+

Shadow Advanced Launchers folder
--------------------------------

The following icons will be available in the Shadow Advanced Launchers folder.

* If an icon in ``Shadow Advanced Launchers`` starts with a number, it is meant to be run in numerical sequence after the lower-numbered icons.
* If an icon in ``Shadow Advanced Launchers`` doesn't start with a number, it can be run independently

The Launch Shadow Right/Left/Bi-manual Hand(s) icon in the main desktop is equivalent to launching:

* 1 - Launch Server Container
* 2 - Launch Server ROSCORE
* 3 - Launch NUC Container and Right/Left/Bi-manual Hands Hardware Control Loop
* 4 - Launch Server Right/Left/Bi-manual GUI

However, with the Shadow Advanced Launcher icons, you can have more granular and customised control of launching different parts of the Shadow software.

.. table::
   :class: tight-table
   
   +--------------------------------------------------+--------------------------------------------------+
   | Icon text                                        |  Icon explanation                                | 
   +==================================================+==================================================+
   | 1 - Launch Server Container                      | Launches the server laptop's docker container    |
   |                                                  |                                                  |
   +--------------------------------------------------+--------------------------------------------------+
   | 2 - Launch Server ROSCORE                        | Launches the ROSCORE inside the server laptop's  |
   |                                                  | docker container                                 |
   +--------------------------------------------------+--------------------------------------------------+
   | 3 - Launch NUC Container and Right Hand Hardware | SSH'es to the NUC, starts its container, and     |
   | Control Loop                                     | launches the right hand realtime control loop    |
   +--------------------------------------------------+--------------------------------------------------+  
   | 3 - Launch NUC Container and Left Hand Hardware  | SSH'es to the NUC, starts its container, and     |
   | Control Loop                                     | launches the left hand realtime control loop     |
   +--------------------------------------------------+--------------------------------------------------+
   | 3 - Launch NUC Container and Bimanual Hands      | SSH'es to the NUC, starts its container, and     |
   | Hardware Control Loop                            | launches the bimanual realtime control loop      |
   +--------------------------------------------------+--------------------------------------------------+
   | 3 - Zero Force Mode - Right Hand                 | Launches the right hand (connected to NUC) in    |
   |                                                  | zero force mode (fingers can be moved easily)    |
   +--------------------------------------------------+--------------------------------------------------+
   | 3 - Zero Force Mode - Left Hand                  | Launches the left hand (connected to NUC) in     |
   |                                                  | zero force mode (fingers can be moved easily)    |
   +--------------------------------------------------+--------------------------------------------------+
   | 4 - Launch Server Right Hand GUI                 | Launches the GUI (Rviz) on server laptop for the |
   |                                                  | right hand                                       |
   +--------------------------------------------------+--------------------------------------------------+
   | 4 - Launch Server Left Hand GUI left hand        | Launches the GUI (Rviz) on server laptop for the |   
   +--------------------------------------------------+--------------------------------------------------+
   | 4 - Launch Server Bimanuals GUI                  | Launches the GUI (Rviz) on server laptop for the |
   |                                                  | bimanual hands                                   |
   +--------------------------------------------------+--------------------------------------------------+
   | Launch Local Shadow Right Hand                   | Launches the right hand (connected to server     |
   |                                                  | laptop) using the same USB-ethernet adapter      |
   +--------------------------------------------------+--------------------------------------------------+
   | Launch Local Shadow Left Hand                    | Launches the left hand (connected to server      |
   |                                                  | laptop) using the same USB-ethernet adapter      |
   +--------------------------------------------------+--------------------------------------------------+
   | Launch Local Shadow Bimanual Hands               | Launches bimanual hands (connected to server     |
   |                                                  | laptop) using the same USB-ethernet adapters     |
   +--------------------------------------------------+--------------------------------------------------+
   | Launch NUC Container                             | SSH'es to the NUC, starts NUC's container and    |
   |                                                  | starts a terminal session inside it              |
   +--------------------------------------------------+--------------------------------------------------+
   | Local Zero Force Mode - Right Hand               | Launches the right hand (connected to server) in |
   |                                                  | zero force mode (fingers can be moved easily)    |
   +--------------------------------------------------+--------------------------------------------------+ 
   | Local Zero Force Mode - Left Hand                | Launches the left hand (connected to server) in  |
   |                                                  | zero force mode (fingers can be moved easily)    |
   +--------------------------------------------------+--------------------------------------------------+
