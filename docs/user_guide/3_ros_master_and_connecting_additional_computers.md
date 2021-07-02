ROS MASTER and connecting additional computers
==============================================

There are 3 computers involved:

Server laptop, hostname: ``serverhostname`` (replace it with the actual hostname in these instructions) (IP: 10.9.11.1)

NUC (IP: 10.9.11.2)

Third computer, hostname: ``thirdcomputerhostname`` (replace it with the actual hostname in these instructions) 

The server laptop acts as the ROS MASTER. To connect additional computers with ROS to server laptop and NUC ROS network (to control and see data from the hand/arm), it is only necessary to have the server laptop and the additional non-Shadow computer with ROS on the same network.

To connect the Third computer to the same network as the server laptop and the NUC, follow these steps:

## I have a Shadow-provided router

Connect an ethernet cable from the Shadow-provided router to the Third computer. The Third computer will then get an IP in the same network as the Server laptop and the NUC (e.g. 10.9.11.5)

IPs will be (for example):

Server laptop (IP: 10.9.11.1)

NUC (IP: 10.9.11.2)

Third computer (IP: 10.9.11.5)

### On the server laptop local machine:

Make sure the server laptop hostname (``serverhostname``) is present exactly twice in the server laptop local machine /etc/hosts:

127.0.0.1 ``serverhostname``

10.9.11.1 ``serverhostname``

Also make sure the Third computer IP is present in this /etc/hosts file:

10.9.11.5 ``thirdcomputerhostname``

### In the server laptop container

You can start a container terminal by clicking on 1 - Launch Server Container in Shadow Advanced Launchers), make sure the name server laptop hostname is also present exactly twice in the /etc/hosts of the container:

127.0.0.1 ``serverhostname``

10.9.11.1 ``serverhostname``

Also make sure the Third computer IP is present in this /etc/hosts file:

10.9.11.5 ``thirdcomputerhostname``

### On the Third computer (and also in the Third computer Docker container if using Docker on the Third computer)

Make sure the third computer hostname (``thirdcomputerhostname``) is present exactly twice in the third computer /etc/hosts:

127.0.0.1 ``thirdcomputerhostname``

10.9.11.5 ``thirdcomputerhostname``

Also make sure the server laptop is present in this /etc/hosts file:

10.9.11.1 ``serverhostname``

Run the following command in the Third computer (or inside its Docker container if using Docker):

```bash
$ export ROS_MASTER_URI=http://serverhostname:11311
```   
 
## I don't have a Shadow-provided router

Connect an ethernet cable from your home/office router to the Server laptop or connect the server laptop to your home/office wifi. The server laptop will acquire a second IP address (e.g. 192.168.1.12). The Third computer will be connected to your home/office network and will have a similar IP address (e.g. 192.168.1.11)

IPs will be (for example):

Server laptop (IP: 10.9.11.1 and 192.168.1.12)

NUC (IP: 10.9.11.2)

Third computer (IP: 192.168.1.11)

### On the server laptop local machine:

Make sure the server laptop hostname (``serverhostname``) is present exactly twice in the server laptop local machine /etc/hosts:

127.0.0.1 ``serverhostname``

192.168.1.12 ``serverhostname``

Also make sure the Third computer IP is present in this /etc/hosts file:

192.168.1.11 ``thirdcomputerhostname``

### In the server laptop container

You can start a container terminal by clicking on 1 - Launch Server Container in Shadow Advanced Launchers), make sure the name server laptop hostname is also present exactly twice in the /etc/hosts of the container:

127.0.0.1 ``serverhostname``

192.168.1.12 ``serverhostname``

Also make sure the Third computer IP is present in this /etc/hosts file:

192.168.1.11 ``thirdcomputerhostname``

### On the Third computer (and also in the Third computer Docker container if using Docker on the Third computer)

Make sure the third computer hostname (``thirdcomputerhostname``) is present exactly twice in the third computer /etc/hosts:

127.0.0.1 ``thirdcomputerhostname``

192.168.1.11 ``thirdcomputerhostname``

Also make sure the server laptop is present in this /etc/hosts file:

192.168.1.12 ``serverhostname``

Run the following command in the Third computer (or inside its Docker container if using Docker):

```bash
$ export ROS_MASTER_URI=http://serverhostname:11311
```   

## Testing

Start the hand using icons on the server laptop. Then, test if the additional computer can see the ROS topics and echo the contents:

```bash
$ rostopic list
```

```bash
$ rostopic echo /joint_states
```  

Now the additional computer is fully connected ROS MASTER of the server laptop.
See the ``Software Description`` > ``Software description of the Hand`` > ``Command line interface``
