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
5. A Stop Sign detection script - ``` stop_sign.py ``` that uses the ONNX version YOLOV3 pre-trained with 80 COCO dataset categories to perform real-time inference and detects the presence of a stop-sign
6. A switching script - ``` siwtch_flag_code.py ``` as a separate node to listen to keypress and publish the flag to switch from Obstacle Avoidance to Line Following Mode 

# How to run the code

In addition to completing the tasks above, we also capture the map of the environment during this process. Therefore, the order in which to execute is as follows:

1. Ensure ROS_MASTER is up and running on the remote PC.
2. Bring up the bot and ensure the sensors are publishing the relevant data topics.
3. Launch the SLAM node to start capturing the map. We used HECTOR SLAM for faster map generation and reduced network load as the algorithm relies only on laser scan data as opposed to the default gmapping method that uses odometry in addition to laser scans.
4. Launch the controller node to complete the wall following/obstacle avoidance course. At the end of the obstacle course, press 's' to switch to line following.
5. Once the run is complete, stop the controller node, save the map file and close the remaining nodes.

## On Gazebo

Naturally, there are differences in environment and parameters between Gazebo and the real world requiring some finetuning specific to each case. 
The scripts with the suffix ``` _gazebo ``` correspond to the simulation run.

1. To run on Gazebo, first launch the world with Gazebo using ``` roslaunch turtlebot3_gazebo turtlebot3_final_autonomy.launch ```. Please note that the ```turtlebot3_final.world ``` must be placed in your Gazebo world file path. Alternatively, you can also place the world file in the world file path of the ``` aue_finals ``` package -> modify the launch file to replace ``` turtlebot3_gazebo ``` with ``` aue_finals``` and then run ``` roslaunch aue_finals turtlebot3_final_autonomy.launch ```
2. Initiate the SLAM node with ``` roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector ```
3. Run ``` roslaunch aue_finals final_Wall_ROSImg_gazebo.launch ``` and press the key ``` s ``` when it's time to switch modes.
4. At the end of the course, terminate the last node and save the map file with ``` rosrun map_server map_saver -f map_gazebo ```

## Real World

As the real world implementation requires an actual turtlebot interfacing, ensure that the ROS_MASTER_URI is set to the same address on both the remote PC and the bot and that they are connected to the same network. The ROS_MASTER_URI must be set to the IP_ADDRESS of the remote PC on this network.

1. Run ``` roscore ``` on the remote PC.
2. Connect to the turtlebot through ssh and run ``` roslaunch turtlebot3_bringup turtlebot3_robot.launch ``` - You will need to update the launch file to initialize an RPi camera node that publishes images to the network along with the odometry and lidar scan data that is published by default. Add the following line to the turtlebot3_robot.launch file to achieve this functionality ``` <node type="raspicam_node" pkg="raspicam_node" name="raspicam_node" output="screen"> ```
3. On the remote PC, launch SLAM with ``` roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector ```
4. Now launch the controller node with ``` roslaunch aue_finals final_Wall_ROSImg.launch ``` and then hit the `s` key to switch to line following once the line is within view.
5. Finally save the map file with ``` rosrun map_server map_saver -f map_real_world ```

# References
Much of the open source code we used is from the official ROBOTIS Turtlebot3 GitHub source. However, there are other sources that need a shout out for helping us piece the puzzle together. Below is the list that should guide you if you get stuck in the process.

1. The Quick Start Guide of Turtlebot3 Burger by ROBOTIS - https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
2. All the open source packages for Turtlebot3 - https://github.com/ROBOTIS-GIT/turtlebot3
3. Demystifying the HSV thresholding for line detection - https://medium.com/programming-fever/how-to-find-hsv-range-of-an-object-for-computer-vision-applications-254a8eb039fc
4. We tested this and did not deploy but an excellent resource for YOLO with ROS - https://github.com/leggedrobotics/darknet ros.
5. Image Zero MQ transport reference, another thing we tested and did not deploy but you will find the code in the repo - https://github.com/jeffbass/imagezmq
6. Pre-trained weights in ONNX for YOLOv3 - https://github.com/onnx/models
7. YOLOv3 inference with ONNX - https://github.com/zxcv1884/yolov3-onnx-inference
8. YOLO pipeline explained - https://towardsdatascience.com/yolo-object-detection-with-opencv-and-python-21e50ac599e9



