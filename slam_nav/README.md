## Objective:
The aim of this exercise is to perform Simultaneous Localization and Mapping(SLAM) on the Turtlebot 3 Burger, both in Gazebo and in real world, using the default gmapping SLAM and Hector SLAM. We will then use the default navigation algorithm on the Turtlebot 3 to navigate to a destination on the generated map and compare the results 
## How to run the code?
Please note that there is no custom code associated with this particular exercise. Follow the below instructions to download the requisite open source packages.

### Downloading Hector SLAM

By installing the default Turtlebot3 packages, you would already have the ```gmapping``` SLAM and the ```navigation``` packages. Hector SLAM, however, must be downloaded separately.

In the remote PC, run the following command:

```sudo apt-get install ros-noetic-hector-slam``` 

Post the installation, source your ROS environment either by directly running ```source opt/ros/noetic/setup.bash``` or through ```source ~/.bashrc``` if you have already added the previous command in your bashrc.


### Running SLAM and Navigation In Gazebo

1. We need the default Turtlebot3 world to be open for the purposes of SLAM.

```roslaunch turtlebot3_gazebo turtlebot3_world.launch``` 

2. Next, in a new terminal we run the gmapping SLAM with ```roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping```. For Hector SLAM, the slam_methods argument will be ```hector``` instead of ```gmapping```

3. This will bring up an RVIZ window displaying the world as seen by the bot through LIDAR. Now manually navigate the turtlebot through the world using teleop and notice the map building slowly. Stop the teleoperation when the world is sufficiently covered.

```roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch```

4. This map can be saved by running ```rosrun map_server map_saver -f myMapFileName```. The map will now be stored to the home directory as both .yaml file and a .pgm file. Now terminate all running operations.

5. In order to use this map file for navigation, we run ```roslaunch turtlebot3_gazebo turtlebot3_world.launch``` followed by ```roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/myMapFileName.yaml```. This will bring up RVIZ once again with the turtlebot projecting an overlaid map over the known map file.
6. At this point, the bot is not sure of its position and might need some landmarks to localize. Using the 2D Pose Estimate option on the RVIZ GUI, select the point on the map where the robot is actually present according to Gazebo and point the arrow in the direction the robot is facing. Now, the robot will move to the specified location but it might be still unsure about its position as indicated by the cloud of green points around it.
7. Initiate a teleop and move the robot around a little bit until the points coalesce to a dense cloud on the robot.
8. Now, you can specify any point on the map using the 2D Nav Goal option on the GUI and point the arrow in the direction of the goal pose. The turtlebot will automatically start navigating to the destination. 

### Running SLAM and Navigation In Real World
Ensure that the Turtlebot and the remote PC are connected to the same Wi-Fi network and export the ```ROS_MASTER_URI``` using the IP addresses.
On the remote PC, open a new terminal and run ```roscore```. In a new terminal, connect to the turtlebot using SSH.

On the second terminal, run ```roslaunch turtlebot3_bringup turtlebot3_robot.launch```
The robot will perform the bringup procedure now. 

For the SLAM and navigation, follow steps 2-8 from the Gazebo procedure but skip the gazebo world launch since that is not required. Note that the 2D-LIDAR on the Turtlebot3 Burger cannot localize very well around dynamic obstacles, therefore create an environment with static obstacles before running SLAM. 

## Results
A key observation even from the Gazebo simulation is that Hector SLAM populates the map faster than the gmapping algorithm. As expected, there is a lot more uncertainty in the robot's position belief in real world compared to the simulated Gazebo world.

### Gazebo
![Gmapping SLAM Video](../videos/Assignment4_recordings_plots/teleop/Circle_tb.gif)
![Gmapping Nav Video](../videos/Assignment4_recordings_plots/teleop/Circle_Teleop_bag.png)
![Hector SLAM Video](../videos/Assignment4_recordings_plots/teleop/Circle_tb.gif)
![Hector Nav Video](../videos/Assignment4_recordings_plots/teleop/Circle_Teleop_bag.png)

### Real world
![Gmapping SLAM Video](../videos/Assignment4_recordings_plots/teleop/Circle_tb.gif)
![Gmapping Nav Video](../videos/Assignment4_recordings_plots/teleop/Circle_Teleop_bag.png)
![Hector SLAM Video](../videos/Assignment4_recordings_plots/teleop/Circle_tb.gif)
![Hector Nav Video](../videos/Assignment4_recordings_plots/teleop/Circle_Teleop_bag.png)
