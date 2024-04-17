# Overview

There are four tasks involved in the successful exectuion of this project:
1. Obstacle Avoidance while following the wall
2. Line Following
3. Detecting the Stop Sign
4. Transitioning from Obstacle Avoidance mode to Line Following Mode

The code is split into four different script files to achieve this functionality.
1. A main integrator program - ``` aue_finals_Wall_Obstacle_ROS_Image_copy.py``` that retains control over motion planning, performs switching from one mode to another
2. An obstacle avoidance program - ``` wall_obstacle_line.py ``` that takes LIDAR scan data and returns desired command velocities to the main program
3. A Line follower prorgam - ``` line.py ``` that takes camera input image and returns desired command velocities to the main program
5. A Stop Sign detection script - ``` live_yolo.py ``` that uses the ONNX version YOLOV3 pre-trained with 80 COCO dataset categories to perform real-time inference and detects the presence of a stop-sign
6. A switching script - ``` siwtch_flag_code.py ``` as a separate node to listen to keypress and publish the flag to switch from Obstacle Avoidance to Line Following Mode 

# How to run the code

## On Gazebo

Naturally, there are differences in environment and parameters between Gazebo and the real world requiring some finetuning specific to each case. 
The scripts with the suffix ``` _gazebo ``` correspond to the simulation run.

To run on Gazebo, first launch the world with Gazebo using ``` roslaunch turtlebot3_gazebo turtlebot3_final_autonomy.launch ```. Please note that the ```turtlebot3_final.world ``` must be placed in your Gazebo world file path. 
Alternatively, you can also place the world file in the world file path of the ``` aue_finals ``` package -> modify the launch file to replace ``` turtlebot3_gazebo ``` with ``` aue_finals``` and then run ``` roslaunch aue_finals turtlebot3_final_autonomy.launch ```

Finally, run ``` roslaunch aue_finals final_Wall_ROSImg_gazebo.launch ``` and press the key ``` s ``` when it's time to switch modes.

## Real World

For the Real World demo, run the following commands:

### On the bot 

``` roslaunch turtlebot3_bringup turtlebot3_robot.launch ``` - The bring up launch file must initialize an RPi camera node that publishes images to the network.

### On the remote PC

``` roslaunch aue_finals final_Wall_ROSImg.launch ``` and then hit the `s` key to switch to line following once the line is within view.
