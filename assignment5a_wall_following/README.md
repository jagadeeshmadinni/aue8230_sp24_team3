# Wall Following:
This package runs the wall following functionality for turtlebot3 in gazebo environment.

## Description

A PD yaw controller has been developed for this functionality with a constant linear velocity. The controller tries to minimise the difference of distance of the bot on the left side and the right side. To better understand the distance of the bot on either side, an average of 75 range values from 15 degrees to 90 degrees on each left and right side are obtained from the LiDAR. This obtained range is compared to get the error and finally, this error is minimised by the PD controller by means of a yaw control i.e. angular velocity in z-direction.

## Advantages:
- A simple algorithm with low complexity.
- Constant linear velocity.

## Disadvantages:
- The tuning of the controller depends not just on the gains of controller but, on how the range values are obtained.
- Tuning of gains might be needed for real-world operation.

## Package description:
```turtlebot3_wall_following.launch``` - Loads the world file for wall following.
```wall_following.launch``` - Runs the algorithm for wall following.
```wall_following.py``` - Pyton based code file for wall following.
![wall_following_video](https://github.com/jagadeeshmadinni/aue8230_sp24_team3/assets/100736973/de42c6b1-fe7f-48d5-9d4b-c67a41638286)
