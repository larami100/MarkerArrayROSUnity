# MarkerArrayROSUnity
Unity application that reads ROS MarkerArray messages and displays them.

The project consists in a ROS application that sends a cube, a sphere, a cylinder and a turtle mesh to a Unity project in order to display them.

The ROS application was developed in C++ in the source file:
/catkin_ws/src/using_markers/src/basic_shapes.cpp

Unity 2019.4.22f1 version is required. 


## Setup steps:

1. In Linux Terminal, go to directory /catkin_ws/

2. Inside that directory, run `catkin_make` command to build using_markers package. 

3. Initialize ROS by executing `roscore` command.

4. With a new terminal window, source the package with the instruction:
   `source /catkin_ws/devel/setup.bash` 

5. In the same terminal window of step 4, run using_markers package:
   `rosrun using_markers basic_shapes`

6. With a new terminal window, run ROSBridge server:
   `roslaunch rosbridge_server rosbridge_websocket.launch`
   
7. Run `ifconfig` command in Terminal and copy IP address.
   
8. Open shapesArrayMarker Unity project.

9. In Hierarchy section, select ROSConnector Game Object. 

10. In Inspector section of that object, change the IP value of Ros Bridge Server Url variable with the one copied of step 7.

11. Run Unity project.



