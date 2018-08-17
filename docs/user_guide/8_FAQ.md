# Frequently Asked Questions

A list of common issues and how to resolve them.

## Hardware Issues

**How do I know when the Hand is powered on?**

* There are lights on the back covers of the fingers. See 3.4
* The fans should be audible

**The Hand is not powered.**

* Check all connections from the power supply to the Hand

**All of the connections are OK. The Hand still won’t power on.**

* Check that the plug socket is turned on and the mains lead is plugged in


**The Hand STILL won’t power on.**

* Get in touch with someone from Shadow: hand@shadowrobot.com


**The Hand is powered but I cannot connect to my PC.**

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

**The Hand doesn’t react after startup of the container**

* Make sure that interface ID is set correctly
* Power cycle the Hand and try again
