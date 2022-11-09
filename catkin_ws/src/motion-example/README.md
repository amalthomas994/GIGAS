**Requirements:**
*  Ubuntu 16.04
*  Install ROS Kinetic [http://wiki.ros.org/kinetic/Installation/Ubuntu](http://wiki.ros.org/kinetic/Installation/Ubuntu)
*  Set up ROS workspace [http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
*  Install Moveit! [https://moveit.ros.org/install/](https://moveit.ros.org/install/)
*  Install ROS-Industrial [http://wiki.ros.org/Industrial/Install](http://wiki.ros.org/Industrial/Install)
*  Download ROS-Industrial package for your robot [https://github.com/ros-industrial/fanuc](https://github.com/ros-industrial/fanuc) 

**Connect to robot:**
check the robot's ip address and change it in the following commands if nessary.
Menu > Setup > Host Comm > TCP/IP

The following command will connect to the IP address of the robot running the ROS software

`roslaunch fanuc_lrmate200ic5l_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=10.16.48.248`

`roslaunch custom_fanuc_w_suction_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=10.16.48.248`

RViz should display a simulation of the robot at its current position

**Control robot:**
RViz has options for controlling the robot using the GUI.
This repo also contains an example of creating an obstacle and moving the robot to a given location. 

To run the example, do the following in your ROS workspace (e.g. /catkin_ws)

`catkin_make`

`rosrun moving_arm moving_arm`
