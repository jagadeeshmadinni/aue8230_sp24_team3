# Overview:
This folder contains the video files (.mov) for task 1 and 2 for assignment 4 of AuE8230 Spring 2024

Along with the videos of turtlebot implementation these folders also contain the following:
	1. Task 1 and 2 contain gifs with screen records of commands required in the implementaion.
	2. Task 2 contain maps files (.pgm and .yaml files) for slam implementaions.
	
# Challenges Faced:
	1. Issue:	Gazebo failed to open.
	   Solution:	Make sure to connect host PC to local network even to run gazebo because for roscore to run successfully, PC should have the IP set in the MASTER_URI.
	   
	2. Issue:	Bot moves sideways in RViz upon giving forward/backward velocity commands​.
	   Solution:	The IMU on the bot was not stationary which made it difficult for the bot to get accurate system state information. Bot stopped moving sideways once IMU was at a fixed position.​
	   
	3. Issue:	Bot moves backward on giving forward velocity commands and vice versa​.
	   Solution:	The initial orientation of the LiDAR initialized the system hence, the orientation of the LiDAR was changed by 180 degrees. Issue was rectified.​
