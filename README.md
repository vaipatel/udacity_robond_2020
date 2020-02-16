# Submissions/Notes for Udacity RoboND 2020

## Gazebo Notes

* Think of Gazebo as a version of Unity or Unreal. Think Entity-Components, basic model editing, yadda yadda.
* It's desigined in a client-server fashion - the [server](http://gazebosim.org/tutorials?tut=components&cat=get_started#GazeboServer) launches the `.world` and performs the simulation, the [client](http://gazebosim.org/tutorials?tut=components&cat=get_started#GraphicalClient) provides rendering and interaction.
* Plugins can extend functionality without the overhead of transport layer, say, to move a robot programmatically without serializing messages. Plugins are typically placed in the `.world` for ease of launching.