# Home Service Robot
## Robotics Software Engineer Nanodegree Program, Udacity
### Project: Home Service Robot

This project demonstrates a TurtleBot2 driving through the environment transporting a number of obstacles. The robot repeteadly requests tasks and in exchange receives the locations of the needed objects. The robot then drives to the assigned location and transports the object to a storage room. Once the robot is informed that there are no more tasks, it drives back home (the center of the apartment).

![HSR-initial](/images/first.png)

#### Requirements
1. **ROS Version: kinetic.** Starting from ROS melodic, the TurtleBot2 is deprecated. I am currently working on migrating this to TurtleBot3.
2. Clone and compile the following ROS packages:
   - [gmapping](http://wiki.ros.org/gmapping)
   - [turtlebot_teleop](http://wiki.ros.org/turtlebot_teleop)
   - [turtlebot_rviz_launchers](http://wiki.ros.org/turtlebot_rviz_launchers)
   - [turtlebot_gazebo](http://wiki.ros.org/turtlebot_gazebo)
3. To run the scripts under __home_service_turtlebot2/home_service_turtlebot2_configs/scripts/__ make sure **xterm** is installed: ``sudo apt-get install -y xterm``.

#### Usage

- A series of scripts to test the robot functionalities are available under __home_service_turtlebot2/home_service_turtlebot2_configs/scripts/__.
- The main system can be started as follows: 

``roscd home_service_turtlebot2_configs/scripts``
``./home_service.sh``

- Each node is then opened in a separate terminal.

#### Workflow

- The **pick_objects** node is responsible for commanding the robot's movement (picking up and dropping off objects).
- The **add_markers** node is in charge of visualizing the objects on RVIZ and managing the tasks. 
- When the system is ready, **pick_objects** sends a mission request to **add_markers**, which in turn assings the next task to the robot. The robot now know the position of the object that must be collected, it drives to it, picks it up and drops it off in the storage room. This is repeated until now more tasks are avaiable.
![HSR-PickUp](/images/second.png)
![HSR-DropOff](/images/third.png)
- The **pick_objects** node is terminated, but the robot remains available for manual commands.
![HSR-Final](/images/final_position.png)

