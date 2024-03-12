## Objective:
The aim of this package is to make a Turtlebot3 Burger track two trajectories using open loop control: a circle and a square
## How to run the code?
Copy the ```assign2b``` directory into your ```catkin_ws/src``` path. Run ```catkin_make```
### Bring up the robot
Ensure that the Turtlebot and the remote PC are connected to the same Wi-Fi network and export the ```ROS_MASTER_URI``` using the IP addresses.
On the remote PC, open a new terminal and run ```roscore```. In a new terminal, connect to the turtlebot using SSH.

On the second terminal, run ```roslaunch turtlebot3_bringup turtlebot3_robot.launch```
The robot will perform the bringup procedure now. Optionally, you can open a third terminal and record the topics published using 
```rosbag record myBagFileName```
Move to the first terminal and run:

  ```roslaunch assign2b circle.launch``` for the circle or

  ```roslaunch assign2b square.launch``` for the square

## Results
The video recordings of the bot traversing the trajectories are in the videos section. We plotted the ```/odom``` data we recorded in the rosbag using MATLAB ROS Bag Viewer.

The results corroborate what we already know about open loop control on a turtlebot. 
### Circle Trajectory
![Circle Video](../videos/Assignment4_recordings_plots/teleop/Circle_tb.gif)
![circle plot](../videos/Assignment4_recordings_plots/teleop/Circle_Teleop_bag.png)
### Square Trajectory
![Square Video](../videos/Assignment4_recordings_plots/teleop/Square_tb.gif)
![square plot](../videos/Assignment4_recordings_plots/teleop/Square_Teleop_bag.png)

The circle is close in shape to a circle even though the bot did not go through the starting point at the end of the first loop.
The square, on the other hand, is a lot more out of shape owing to the following reasons:
  1. Inertia of the vehicle while turning

  2. Coefficient of friction with the ground is not constant making it more difficult for the bot to navigate

  3. Irregular torque split between the two wheels because of manufacturing irregularities.
