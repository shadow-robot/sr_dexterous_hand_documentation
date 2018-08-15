A list of common issues and how to resolve them.

## Hardware Issues

**How do I know when the Grasper is powered on?**

* There are lights on the back covers of the fingers. See 3.4
* The fans should be audible

**The Grasper is not powered.**

* Check all connections from the power supply to the Grasper

**All of the connections are OK. The Grasper still won’t power on.**

* Check that the plug socket is turned on and the mains lead is plugged in


**The Grasper STILL won’t power on.**

* Get in touch with someone from Shadow: grasper@shadowrobot.com


**The Grasper is powered but I cannot connect to my PC.**

* Check the ethernet cable connections
* Check the link light on your computer’s ethernet port (fixed state, not flashing)
* Check you’re using the right interface (see 9.2 ROS API Issues)

## Software Issues
### Docker

**I am getting following error when trying to pull the image:**
```bash
Error response from daemon: pull access denied for shadowrobot/flexible-hand:kinetic-release, repository does not exist or may require 'docker login'
```

Make sure that you are logged in to docker on your host machine using:
```bash
$ docker login
```
and type your credentials.

**I am getting following error when trying to run the image:**
```bash
docker: Error response from daemon: Conflict. The container name [name] is already in use
```
A container with your name already exists. Either remove the previous container using docker rm [name] or use docker start [name] to start the existing container. You can check the currently available containers, either running or stopped, using:

```bash
$ docker ps -a
```

**I am not getting any output from the Grasper in the console**

The verbose flag was missing when running the docker. Remove container and make sure you specify “-e verbose=true” in the docker run command.

**The Grasper doesn’t react after startup of the container**

* Make sure that interface ID is set correctly
* Power cycle the Grasper and try again

## Communicating with the Grasper

**I am getting the following error:**

```bash
Error: start: Cannot start container [container_id]: Port already in use: 8080
```

Port 8080 is used by RESTful API. Make sure that you are not already running another container using this port or that you are not occupying it in any other way.

**I am getting the following error when sending a grasp:**

There was a problem publishing the grasp command.
Check that the name of the grasp is correct and that the grasp is available in the database running the following command:

```bash
$ curl -i -H "Accept: application/json" -H "Content-Type: application/json" -X GET http://0.0.0.0:8080/grasps_available
```