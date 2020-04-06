# Submissions/Notes for Udacity RoboND 2020

- [Gazebo Notes](#gazebo-notes)
- [ROS Notes](#ros-notes)
  - [Computation Graph Level](#computation-graph-level)
    - [What the developer develops](#what-the-developer-develops)
    - [What ROS provides](#what-ros-provides)
  - [File System Level](#file-system-level)

## Gazebo Notes

* Think of Gazebo as a version of Unity or Unreal. Think Entity-Components, basic model editing, yadda yadda.
* It's desigined in a client-server fashion - the [server](http://gazebosim.org/tutorials?tut=components&cat=get_started#GazeboServer) launches the `.world` and performs the simulation, the [client](http://gazebosim.org/tutorials?tut=components&cat=get_started#GraphicalClient) provides rendering and interaction.
* Plugins can extend functionality without the overhead of transport layer, say, to move a robot programmatically without serializing messages. Plugins are typically placed in the `.world` for ease of launching.
* Gazebo can be integrated with ROS. TODO: Expand on this. Like how it can be done through plugins.

## ROS Notes

Think of ROS as a framework for creating a Computation Graph (not the DNN kind) where [Nodes](http://wiki.ros.org/Nodes) can pass [Messages](http://wiki.ros.org/Messages) to each other and process them. One Node in a robot model might [publish odometry data](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_planar_move.cpp#L305), another might [subscribe to odometry and other data and apply an extended kalman filter](https://github.com/udacity/robot_pose_ekf/blob/master/src/odom_estimation_node.cpp#L425-L426) for localization.

### Computation Graph Level

ROS provides a framework to develop/run a Computation Graph like architecture where various pieces of computation can run and interact with each other. Here's a look at how things work.

#### What the developer develops
* [Nodes](http://wiki.ros.org/Nodes) which i) pass data peer-to-peer to other Nodes, and ii) process the data (obviously).
* [Messages](http://wiki.ros.org/Messages) which are data structures used by Nodes to communicate. 
  * Nodes do many-to-many sending/receiving of Messages by Publishing/Subscribing to [Topics](http://wiki.ros.org/Topics).
  * Nodes do one-to-one sending/receiving of Messages by Requesting/Replying to [Services](http://wiki.ros.org/Services).

Of course ROS also provides many Nodes, Messages, etc. out-of-the-box. But for the most part, this is the part of the ROS framework that developers develop for their specific needs.

#### What ROS provides

ROS provides the machinery to run Nodes and connect them via Messaging capabilities like Topic pub/sub, Service invocation etc.

* An [XMLRPC server](https://github.com/ros/ros_comm/blob/kinetic-devel/tools/rosmaster/src/rosmaster/master.py#L66) [(base impl](https://github.com/ros/ros_comm/blob/kinetic-devel/tools/rosgraph/src/rosgraph/xmlrpc.py#L83), [wrapper)](https://github.com/ros/ros_comm/blob/kinetic-devel/tools/rosgraph/src/rosgraph/xmlrpc.py#L153) called [Master](http://wiki.ros.org/Master). It's like a registrar for Nodes. It helps Nodes look each other up through their interest in publishing/subscribing to Topics.
* A [Parameter Server](http://wiki.ros.org/Parameter%20Server), which is a [thread-safe](https://github.com/ros/ros_comm/blob/kinetic-devel/tools/rosmaster/src/rosmaster/paramserver.py#L63) [dictionary](https://github.com/ros/ros_comm/blob/kinetic-devel/tools/rosmaster/src/rosmaster/paramserver.py#L64) of global param values. Not meant to store heavy data, just some static stuff needed communally by various Nodes.
* TODO: Bags

This part of ROS is to be thought of as closed for a typical developer.

### File System Level

The Computation Graph Level gives us an idea of what ROS developers typically develop. It begs the question - is there a project framework that helps develop typical ROS components like Nodes, Messages, Services etc.?

Yes there is! The official way to develop a ROS project is via the [catkin build system](http://wiki.ros.org/catkin).

At the Filesystem level, a catkin ROS project will consist of a [workspace](http://wiki.ros.org/catkin/workspaces) folder. This workspace layout is per the catkin convention and basically mimics an out-of-source CMake style build structure. 

In particular, inside the `src` folder will be folders for one or more [packages](http://wiki.ros.org/catkin/Tutorials/CreatingPackage). Packages are collections of ROS comp graph items and other resources like models, `.launch` environments, etc. The official build system for building packages is the catkin build system. The workspace directory is referred to as the catkin workspace.

A typical package, say identified by a folder `my-package`, may be laid out as follows.

`my-package/`
* `package.xml`: Mandatory. Needed for meta-info like version, dependencies, etc.
* `CMakeLists.txt`: Mandatory (I think). Needed for finding dependencies, message/service definitions, other libraries etc. Like normal CMakeLists except tailored toward ROS development.
* `src/*.cpp` or `src/*.py`: source code for Node executables which can publish/subscribe to Topics to receive/send/process data in Messages
* `msg/*.msg`: custom Message definition files
* `srv/*.srv`: custom Service definition files
* `launch/*.launch`: launch files
* `world/*.world`: world files
