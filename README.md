# Overview:
This repository hosts the code and demo results for our team in the project based learning course Autonomy Science & Systems. The course builds up blocks of the Sense-Think-Act paradigm through application on the widely used Turtlebot3 educational robot platform leading up to a captstone project resulting in a semi/fully autonomous navigation of the bot in a defined course. Below is a sample video of some of the cool stuff we got to do in this class.

![Video](https://github.com/jagadeeshmadinni/aue8230_sp24_team3/blob/1600218558c0d3158436a43ccbad74b1a64c4fd3/aue_finals/Complete_run_2x_speed.mp4)

## Team Members:
1. Geeta Koduri
2. Jagadeesh Madinni
3. Johir Suresh
4. Saumil Pradhan

The entire repo is composed of ROS(Robot Operating System) packages that deliver specific functionalities. We used the ROS Noetic distribution with most coding in Python. To achieve modular efficiency, the repo only contains package level data. In order to reproduce the results, you can download the packages, place them under your ```catkin_ws/src``` directory and run ```catkin_make```. For example, to run the ```assign2b``` package, first ```git clone``` this repo or download the source code. Next, copy the ```assign2b``` directory to the ```src``` of your catkin workspace and then run ```catkin_make```.


## Contents by package:
```assign2b``` - Openloop circle and square trajectory following on a Turtlebot3.

```slam_nav``` - Simultaneous Localization and Mapping(SLAM) using Gmapping and Hector SLAM algorithms followed by navigation in both Gazebo and real world on a Turtlebot3 Burger.

```assignment5a_wall_following``` - A wall following Turtlebot3 implementation in a simulated Gazebo world using LIDAR scan data.

```assignment5b_obstacleavoidance``` - Obstacle Avoidance implementation on a Turtlebot3 running indefinitely in a simulated world.

```assignment5c_visualtrackingandfollowing``` - Line-following behavior for a Turtlebot3 robot in a simulated Gazebo environment.

```aue_finals``` - Final class project where our Turtlebot navigates an obstacle course within walls, switches to following a line and comes to a 3 second stop upon detecting a stop-sign.

Specific instructions on each package are inside each directory.
