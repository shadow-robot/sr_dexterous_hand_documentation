## First time users
If you are unfamiliar with ROS and intend to use the ROS API, it is highly recommended that you read the [ROS Tutorials](http://www.ros.org/wiki/ROS/Tutorials).

If you are unfamiliar with the terminal on Linux, you should look [here](https://askubuntu.com/questions/183775/how-do-i-open-a-terminal).

Shadow software is deployed using Docker. Docker is a container framework where each container image is a lightweight, stand-alone, executable package that includes everything needed to run it. It is similar to a virtual machine but with much less overhead. Follow the instructions in the next section to get the latest Docker container of the grasper driver and interface up and running.

## Docker
### Installing the software on a new PC using the one-liner

We have created a one-liner that is able to install Docker, download the image and create a new container for you. It will also create two desktop icons, one to start the container and launch the hand and another one to save the log files locally. To use it, you first need to have a PC with Ubuntu installed on it (preferable version 16.04) then follow these steps: 


#### Check your Grasper interface ID

Before setting up the docker container, the EtherCAT interface ID for the Grasper needs to be discovered. In order to do so, after plugging the Grasper’s ethernet cable into your machine and powering it up, please run

```bash
$ sudo dmesg
```

command in the console. At the bottom, there will be information similar to the one below:
   
```bash
[490.757853] IPv6: ADDRCONF(NETDEV_CHANGE): enp0s25: link becomes ready
```

In the above example, ‘enp0s25’ is the interface ID that is needed.

#### Get a docker ID

You need to create a Docker ID here (if you do not have one already): [https://hub.docker.com/](https://hub.docker.com/)

Then send an email with your ID to grasper@shadowrobot.com so we can grant you permission to pull the image.

#### Run the one-liner:

The one-liner will install Docker, pull the image from Docker Hub, and create and run a container with the parameters specified. In order to use it, use the following command:

```bash
bash <(curl -Ls http://bit.do/launch-sh) -i shadowrobot/flexible-hand:kinetic-release -n modular-grasper -sn Grasper_Launcher -e [EtherCAT interface ID]
```

You will need to specify the EtherCAT interface ID that you found in the previous step. You can also add -r true in case you want to reinstall the docker image and container. It will prompt you for your Docker username and password before pulling the image. When it finishes it will show:

```bash
Operation completed
```
and it will create two desktop icons on your desktop that you can double-click to launch the Grasper or save the log files from the active containers to your desktop.
The icon that launches the Grasper looks like this:

![desktop_icon](../img/desktop_icon.png)
   
And for saving the logs:

![log_icon](../img/log_icon.png)

### Using a PC that Shadow provided
In this case, the previous steps would have been performed by the Shadow team before, then the only thing to do to start the Grasper is to either double-click the desktop icon or to run the container using:

```bash
$ docker start modular-grasper
```

You can check the currently available containers using:
```bash
$ docker ps -a
```

The container will be ready when fingers move to the zero position. 

## Communicating with the Grasper

### Introduction

When the container is started, a RESTful API server is running to allow communication with the grasper. We currently have the following API methods available:


URL | Description | Method
--- | --- | ---
/joint_positions | Get the position for all joints of a robot | GET
/joint_position?joint_name=name | Get position for a specific joint | GET
/grasp | Download grasp definition(s) | GET
/joint_positions | Set joint target positions | POST
/select_grasp | Select a grasp state | POST
/add_grasps | Import a grasp | POST
/delete_grasps | Delete grasp(s) | POST
/open_joint_sliders | Open the joint sliders rqt plugin | GET
/open_change_controllers | Open the change controllers rqt plugin | GET

### Using a browser
In order to use the API from your browser, please run the container and, in a host browser, type the following in the address bar: [http://localhost:8080/help](http://localhost:8080/help).

There will be four tabs available:

<ul>
<li><b>API Overview</b>: It provides useful instructions on how to use the RESTful API. 
<img src="https://raw.githubusercontent.com/shadow-robot/fh_documentation/master/docs/img/API_overview_screenshot.png" alt=""></li>
<li><b>Grasps</b>: It checks the grasp database and provides information on the available grasps that are currently stored. You can import, export and delete grasps using the different buttons available.
<img src="https://raw.githubusercontent.com/shadow-robot/fh_documentation/master/docs/img/API_grasps_screenshot.png" alt=""></li>
<li><b>Execute grasps</b>: It provides a graphical interface for executing grasps on the grasper.
<img src="https://raw.githubusercontent.com/shadow-robot/fh_documentation/master/docs/img/API_execute_grasps_screenshot.png" alt=""></li>
<li><b>Create grasp</b>: It provides a graphical interface for saving new grasps.
<img src="https://raw.githubusercontent.com/shadow-robot/fh_documentation/master/docs/img/API_create_grasp_screenshot.png" alt=""></li>
</ul>

In order to perform a grasp, please follow these instructions:


|    | Action | Grasper reaction|
|--- | --- | --- |
|1   | Choose a grasp from the Select a grasp drop down | Grasper will immediately go to pre-grasp position |
|2   | Choose the maximum torque to apply. You can use either the slider or the edit box | - |
|3   | Place an object inside of the grasper and click Execute | Grasper will move to the grasp pose and squeeze to grasp the object |

### Using bash commands or python scripts

The RESTful API can also be used with bash commands and python scripts. Detailed information on how to use each function is available in the [API Overview](http://localhost:8080/help) webpage detailed in the previous section. You will also need to know the grasp ID - this can be found in the [Grasps available](http://localhost:8080/grasps) webpage. An example of a bash command to execute the pre-grasp for the handH_tripod_power grasp can be seen below:

```bash
$ curl -X POST -H "Content-Type: application/json" -d '[{"grasp_id": "handH_tripod_power", "grasp_state": "pre_grasp", "max_torque": 100}]' http://0.0.0.0:8080/select_grasp
```

Analogous example using python:

```python
import json
import urllib2

grasp_cmd = [{"grasp_id": "handH_grasp_1", "grasp_state": "pre_grasp", "max_torque": 100}]
req = urllib2.Request('http://0.0.0.0:8080/select_grasp')
req.add_header('Content-Type', 'application/json')
response = urllib2.urlopen(req, json.dumps(grasp_cmd))
```

## Grasper Status
The modular-grasper provides a graphical interface for monitoring the robot state using diagnostic messages. In order to use it, after launching the Grasper, run the following command in a separate terminal:

```bash
$ rqt
```

Then, in the top left corner select Plugins and from there, Robot Tools>Diagnostics Viewer, as in the following screenshot:

![grasper_status_1](../img/grasper_status_1.png)

When the Diagnostics Viewer is launched, you will be able to see the following three fields:
<ul>
<li>EtherCat</li>
<li>Realtime Control Loop</li>
<li>Shadow Hand</li>
</ul>

![grasper_status_2](../img/grasper_status_2.png)

Every field is divided into sections and subsections. By double-clicking a subsection, a window pops up with the corresponding information. In the bottom of the Diagnostics Viewer there is a timeslot bar that shows the status of the Grasper. It should look green when everything is working correctly and change to yellow or red when there are some issues that need attention. 

The **EtherCat** field is divided in the **Ethercat Master** and Ethercat Slaves. The EtherCat Master subsection contains information regarding the EtherCat communication such as connected devices and RX and TX times and errors.

![grasper_status_3](../img/grasper_status_3.png)

The EtherCat Device subsection contains general information regarding the EtherCat ports of the robot.

![grasper_status_4](../img/grasper_status_4.png)

The **H#_F#_Ethercat subsections** contain specific EtherCat information for the specific fingers of the grasper, such as firmware details (release number etc), product code and port status.

![grasper_status_5](../img/grasper_status_5.png)

The **Realtime Control Loop** subsection contains information regarding the real-time control loop of the driver.

![grasper_status_6](../img/grasper_status_6.png) 

The **Shadow Hand** field contains different subsections for each finger connected to the hand. Each finger contains subsections for each of its motors with information such as temperature, current and effort.

![grasper_status_7](../img/grasper_status_7.png)

## Saving log files
When running the one-liner, along with the icon that starts the Grasper, you will also notice a second icon named Save logs that is used to retrieve and copy all the available logs files from the active containers locally on your Desktop. This icon will create a folder that matches the active container's name and the next level will include the date and timestamp it was executed. When it starts, it will prompt you if you want to continue, as by pressing yes it will close all active containers. If typed 'y' to continue, you will have to enter a description of the logging event and will start coping the bag files, logs and configuration files from the container and then exit. Otherwise, the window will close and no further action will happen.
