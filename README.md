# Submissions/Notes for Udacity RoboND 2020

## Gazebo Notes

* Think of Gazebo as a version of Unity or Unreal. Think Entity-Components, basic model editing, yadda yadda.
* It's desigined in a client-server fashion - the [server](http://gazebosim.org/tutorials?tut=components&cat=get_started#GazeboServer) launches the `.world` and performs the simulation, the [client](http://gazebosim.org/tutorials?tut=components&cat=get_started#GraphicalClient) provides rendering and interaction.
* Plugins can extend functionality without the overhead of transport layer, say, to move a robot programmatically without serializing messages. Plugins are typically placed in the `.world` for ease of launching.
* Gazebo can be integrated with ROS. TODO: What does this mean?

## ROS Notes

Think of ROS as a framework for creating a Computation Graph (not the DNN kind) where Nodes can pass Messages to each other and process them. One Node in a robot model might [publish odometry data](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/5e5bf8469ddf907b5ffbe1534c0ceee10e562625/gazebo_plugins/src/gazebo_ros_planar_move.cpp#L238), another might [subscribe to odometry and other data and apply an extended kalman filter](https://github.com/udacity/robot_pose_ekf/blob/master/src/odom_estimation_node.cpp#L425-L426) for localization.

* At the Computation Graph level, ROS consists of 
  * Nodes which directly pass Messages to other Nodes by Pub-Subing to Topics, and "do" things like processing messages.
* At the Filesystem level, ROS is made of packages, which consist of 
  * a package.xml for meta-info like version, deps, etc., 
  * executables called Nodes which can publish/subscribe to Topics to receive/send/process data in Messages,
  * custom Message and Service definitions, launch files etc.
