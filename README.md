# Submissions/Notes for Udacity RoboND 2020

## Gazebo Notes

* Think of Gazebo as a version of Unity or Unreal. Think Entity-Components, basic model editing, yadda yadda.
* It's desigined in a client-server fashion - the [server](http://gazebosim.org/tutorials?tut=components&cat=get_started#GazeboServer) launches the `.world` and performs the simulation, the [client](http://gazebosim.org/tutorials?tut=components&cat=get_started#GraphicalClient) provides rendering and interaction.
* Plugins can extend functionality without the overhead of transport layer, say, to move a robot programmatically without serializing messages. Plugins are typically placed in the `.world` for ease of launching.
* Gazebo can be integrated with ROS. TODO: Expand on this. Like how it can be done through plugins.

## ROS Notes

Think of ROS as a framework for creating a Computation Graph (not the DNN kind) where [Nodes](http://wiki.ros.org/Nodes) can pass [Messages](http://wiki.ros.org/Messages) to each other and process them. One Node in a robot model might [publish odometry data](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/5e5bf8469ddf907b5ffbe1534c0ceee10e562625/gazebo_plugins/src/gazebo_ros_planar_move.cpp#L305), another might [subscribe to odometry and other data and apply an extended kalman filter](https://github.com/udacity/robot_pose_ekf/blob/master/src/odom_estimation_node.cpp#L425-L426) for localization.

### Computation Graph Level

At the Computation Graph level, ROS consists of

* [Nodes](http://wiki.ros.org/Nodes) which i) pass data peer-to-peer to other Nodes, and ii) process the data (obviously).
* [Messages](http://wiki.ros.org/Messages) which are data structures used by Nodes to communicate. 
  * Nodes do many-to-many sending/receiving of Messages by Publishing/Subscribing to [Topics](http://wiki.ros.org/Topics).
  * Nodes do one-to-one sending/receiving of Messages by Requesting/Replying to [Services](http://wiki.ros.org/Services).
* An [XMLRPC server](https://github.com/ros/ros_comm/blob/melodic-devel/tools/rosmaster/src/rosmaster/master.py#L66) [(base impl](https://github.com/ros/ros_comm/blob/902fb00fc7b2e881575a270a99c3a077ba5cdbba/tools/rosgraph/src/rosgraph/xmlrpc.py#L87), [wrapper)](https://github.com/ros/ros_comm/blob/902fb00fc7b2e881575a270a99c3a077ba5cdbba/tools/rosgraph/src/rosgraph/xmlrpc.py#L163) called [Master](http://wiki.ros.org/Master). It's like a registrar for Nodes. It helps Nodes look each other up through their interest in publishing/subscribing to Topics.
* A [Parameter Server](http://wiki.ros.org/Parameter%20Server), which is a [thread-safe](https://github.com/ros/ros_comm/blob/89ca9a7bf288b09d033a1ab6966ef8cfe39e1002/tools/rosmaster/src/rosmaster/paramserver.py#L63) [dictionary](https://github.com/ros/ros_comm/blob/89ca9a7bf288b09d033a1ab6966ef8cfe39e1002/tools/rosmaster/src/rosmaster/paramserver.py#L64) of global param values. Not meant to store heavy data, just some static stuff needed communally by various Nodes.
* TODO: Bags

### File System Level
At the Filesystem level, ROS is made of packages, which consist of 
* a package.xml for meta-info like version, dependencies, etc.
* executables for Nodes which can publish/subscribe to Topics to receive/send/process data in Messages
* custom Message definition files
* custom Service definition files
* launch files
