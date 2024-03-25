# ROS Package: assignment5c_visualtrackingandfollowing

This ROS package implements a line-following behavior for a Turtlebot3 robot in a simulated Gazebo environment. The package consists of two main files:

- **turtlebot3_follow_line.launch**: This launch file sets up the Gazebo simulation environment and spawns the Turtlebot3 robot model into it. It includes the necessary parameters for simulation, such as the world file, robot model type, and initial pose. Additionally, it launches the `follow_line_step_hsv.py` node to initiate the line-following behavior.

- **follow_line_step_hsv.py**: This Python script implements the line-following algorithm using computer vision techniques. It subscribes to the RGB camera feed of the Turtlebot3 and processes the images to detect a line using color thresholding in HSV color space. It calculates the centroid of the detected line and computes an angular velocity command to keep the robot following the line.

## Usage

1. **Clone this repository into your ROS workspace:**
   ```bash
   git clone https://github.com/your_username/Assignment5c_visualtrackingandfollowing.git

2. **Build your ROS workspace:**
   ```bash
   cd /path/to/your/workspace catkin build

3. **Activate workspace:**
   ```bash
   cd /path/to/your/workspace source devel/setup.bash

4. **Launch the simulation environment and the line-following node:**
   ```bash
   roslaunch assignment5c_visualtrackingandfollowing turtlebot3_follow_line.launch

## Details

- **turtlebot3_follow_line.launch**

    This launch file sets up the Gazebo simulation environment and spawns the Turtlebot3 robot model.
    It includes parameters such as the world file, robot model type, and initial pose.
    It launches the follow_line_step_hsv.py node to initiate the line-following behavior.

- **follow_line_step_hsv.py**

    This Python script implements the line-following algorithm using computer vision techniques.
    It subscribes to the RGB camera feed of the Turtlebot3 and processes the images to detect a line using color thresholding in HSV color space.
    It calculates the centroid of the detected line and computes an angular velocity command to keep the robot following the line.
    Calculates the error term for maintaining lane position.

## Dependencies

This ROS package relies on the following dependencies:

- **OpenCV**: OpenCV (Open Source Computer Vision Library) is used for image processing and computer vision tasks. It is required for the `follow_line_step_hsv.py` node to detect lines in the camera feed and perform line-following behavior.


# Installing OpenCV2 and Simulation files

## OpenCV Installation

- **OpenCV (Open Source Computer Vision Library)** is an open-source library containing several computer vision algorithms. We will be making use of this library to work with the video stream from our camera.

- **We will make use of the OpenCV2 library. To install it, use the following command:**

   ```bash
   $ sudo apt update
   $ sudo apt install libopencv-dev python3-opencv
   
- **Verify the installation by importing cv2 and printing out the version number:**
  ```bash
  $ python3 -c "import cv2; print(cv2.__version__)"


## Simulation Files Installation

  ### To install simulation files, follow these steps:

- **Clone the vision_opencv repository:**

  ```bash
  $ git clone -b noetic https://github.com/ros-perception/vision_opencv.git

- **Navigate to your workspace directory (name_of_your_ws) and build the package:**

  ```bash
  $ cd ~/name_of_your_ws && catkin_make

    
## Video

A video file named assignment5c_visualtrackingandfollowing.mp4 is included in the video folder of this package. 

