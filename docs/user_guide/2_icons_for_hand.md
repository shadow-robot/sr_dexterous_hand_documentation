# Desktop icons for the hand

The following icons will be available on the server laptop desktop for launching and controlling the right, left and bimanual hands. Please note that in Ubuntu 20.04 these icons will be located inside a folder called 'Shadow Launcher' on the desktop. You might need to right Click with your mouse and select 'Allow Launching'

## Main desktop icons

```eval_rst
.. table::
   :class: tight-table
   
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | Icon picture                                     | Icon text                                        |  Icon explanation                                | 
   +==================================================+==================================================+==================================================+
   | .. image:: ../img/log-icon.png                   | Shadow ROS Logs Saver and Uploader               | Saves ROS logs from server and NUC, uploads them |
   |    :width: 100                                   |                                                  | to Shadow servers and emails them to Shadow      |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e.png                     | Launch Shadow Right Hand                         | Launches the right hand (container, ROS core,    |
   |    :width: 100                                   |                                                  | NUC hardware control loop, server GUI)           |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e-left.png                | Launch Shadow Left Hand                          | Launches the left hand (container, ROS core,     |
   |    :width: 100                                   |                                                  | NUC hardware control loop, server GUI)           |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e-bimanual.png            | Launch Shadow Bimanual Hands                     | Launches the both hands (container, ROS core,    |
   |    :width: 100                                   |                                                  | NUC hardware control loop, server GUI)           |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/ROS_logo.png                   | Shadow NUC RQT                                   | Once the hand has been launched, this icon       |
   |    :width: 100                                   |                                                  | starts ROS RQT inside the NUC's docker container |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/documentation_icon.png         | Dexterous Hand Documentation                     | Opens online documentation if internet connected |
   |    :width: 100                                   |                                                  | or offline documentation if no internet          |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/close_icon.png                 | Shadow Close Everything                          | Cleanly stops ROS processes, closes containers,  |
   |    :width: 100                                   |                                                  | and closes all Shadow related terminals          |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+

```
## Shadow Demos folder

The following icons will be available in the Shadow Demos folder. They will only work once the hand has been launched.

```eval_rst
.. table::
   :class: tight-table
   
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | Icon picture                                     | Icon text                                        |  Icon explanation                                | 
   +==================================================+==================================================+==================================================+
   | .. image:: ../img/close-hand-icon-left.png       | Close Left Hand                                  | Once the hand has been launched, this icon will  |
   |    :width: 100                                   |                                                  | close (pack) the left hand                       |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/close-hand-icon-right.png      | Close Right Hand                                 | Once the hand has been launched, this icon will  |
   |    :width: 100                                   |                                                  | close (pack) the right hand                      |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/close-hand-icon-bimanual.png      | Close Bimanual Hands                          | Once bimanual hands have been launched, this icon will  |
   |    :width: 100                                   |                                                  | close (pack) both hands                     |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/demo-hand-icon-left.png        | Biotac Demo Left Hand/Demo Left Hand             | Once the hand has been launched, this icon will  |
   |    :width: 100                                   |                                                  | run various (tactile/keyboard-actived) left hand demos    |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/demo-hand-icon-right.png       | Biotac Demo Right Hand/Demo Right Hand           | Once the hand has been launched, this icon will  |
   |    :width: 100                                   |                                                  | run various (tactile/keyboard-actived) right hand demos   |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/demo-hand-icon-bimanual.png    | Biotac Demo Bimanual Hands/Demo Bimanual Hands   | Once bimanual hands have been launched, this icon will  |
   |    :width: 100                                   |                                                  | run various (tactile/keyboard-actived) bimanual hands demos   |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/open-hand-icon-left.png        | Open Left Hand                                   | Once the hand has been launched, this icon will  |
   |    :width: 100                                   |                                                  | fully open the left hand                         |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/open-hand-icon-right.png       | Open Right Hand                                  | Once the hand has been launched, this icon will  |
   |    :width: 100                                   |                                                  | fully open the right hand                        |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/open-hand-icon-bimanual.png    | Open Bimanual Hands                              | Once bimanual hands have been launched, this icon will  |
   |    :width: 100                                   |                                                  | fully open both hands                       |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
```

## Shadow Advanced Launchers folder

The following icons will be available in the Shadow Advanced Launchers folder.

* If an icon in ``Shadow Advanced Launchers`` starts with a number, it is meant to be run in numerical sequence after the lower-numbered icons.
* If an icon in ``Shadow Advanced Launchers`` doesn't start with a number, it can be run independently

The Launch Shadow Right/Left/Bimanual Hand(s) icon in the main desktop is equivalent to launching:

* 1 - Launch Server Container
* 2 - Launch Server ROSCORE
* 3 - Launch NUC COntainer and Right/Left/Bimanual Hands Hardware Control Loop
* 4 - Launch Server Right/Left/Bimanual GUI

However, with the Shadow Advanced Launcher icons, you can have more granular and customised control of launching different parts of the Shadow software.

```eval_rst
.. table::
   :class: tight-table
   
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | Icon picture                                     | Icon text                                        |  Icon explanation                                | 
   +==================================================+==================================================+==================================================+
   | .. image:: ../img/laptop.jpg                     | 1 - Launch Server Container                      | Launches the server laptop's docker container.   |
   |    :width: 100                                   |                                                  |                                                  |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/ROS_logo.png                   | 2 - Launch Server ROSCORE                        | Launches the ROSCORE inside the server laptop's  |
   |    :width: 100                                   |                                                  | docker container                                 |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e.png                     | 3 - Launch NUC Container and Right Hand Hardware | SSH'es to the NUC, starts its container, and     |
   |    :width: 100                                   | Control Loop                                     | launches the right hand realtime control loop    |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e-left.png                | 3 - Launch NUC Container and Left Hand Hardware  | SSH'es to the NUC, starts its container, and     |
   |    :width: 100                                   | Control Loop                                     | launches the left hand realtime control loop     |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e-bimanual.png            | 3 - Launch NUC Container and Bimanual Hands      | SSH'es to the NUC, starts its container, and     |
   |    :width: 100                                   | Hardware Control Loop                            | launches the bimanual realtime control loop      |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e.png                     | 3 - Zero Force Mode - Right Hand                 | Launches the right hand (connected to NUC) in    |
   |    :width: 100                                   |                                                  | zero force mode (fingers can be moved easily)    |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e-left.png                | 3 - Zero Force Mode - Left Hand                  | Launches the left hand (connected to NUC) in     |
   |    :width: 100                                   |                                                  | zero force mode (fingers can be moved easily)    |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/rviz.png                       | 4 - Launch Server Right Hand GUI                 | Launches the GUI (Rviz) on server laptop for the |
   |    :width: 100                                   |                                                  | right hand                                       |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/rviz.png                       | 4 - Launch Server Left Hand GUI                  | Launches the GUI (Rviz) on server laptop for the |
   |    :width: 100                                   |                                                  | left hand                                        |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/rviz.png                       | 4 - Launch Server Bimanuals GUI                  | Launches the GUI (Rviz) on server laptop for the |
   |    :width: 100                                   |                                                  | bimanual hands                                   |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e.png                     | Launch Local Shadow Right Hand                   | Launches the right hand (connected to server     |
   |    :width: 100                                   |                                                  | laptop) using the same USB-ethernet adapter      |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e-left.png                | Launch Local Shadow Left Hand                    | Launches the left hand (connected to server      |
   |    :width: 100                                   |                                                  | laptop) using the same USB-ethernet adapter      |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e-bimanual.png            | Launch Local Shadow Bimanual Hands               | Launches bimanual hands (connected to server     |
   |    :width: 100                                   |                                                  | laptop) using the same USB-ethernet adapters     |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/nuc.png                        | Launch NUC Container                             | SSH'es to the NUC, starts NUC's container and    |
   |    :width: 100                                   |                                                  | starts a terminal session inside it              |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e.png                     | Local Zero Force Mode - Right Hand               | Launches the right hand (connected to server) in |
   |    :width: 100                                   |                                                  | zero force mode (fingers can be moved easily)    |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+
   | .. image:: ../img/hand-e-left.png                | Local Zero Force Mode - Left Hand                | Launches the left hand (connected to server) in  |
   |    :width: 100                                   |                                                  | zero force mode (fingers can be moved easily)    |
   +--------------------------------------------------+--------------------------------------------------+--------------------------------------------------+

```
