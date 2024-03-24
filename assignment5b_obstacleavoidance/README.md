**Obstacle Avoidance - Emergency Maneuver**

The '''obstacleAvoidance.py''' code works as a stand-alone code to avoide obstacles for a turtlebot that runs on gazebo. This algorithm makes the turtlebot run indefinitely avoiding obstacles, until the user stops. This algorithm is designed to be combined with the motion planning algorithm for the project, this can avoid obstacles that are not mapped during the SLAM process. This runs a discrete time, discrete input and continuous state optimal controller that maps LIDAR distances in various angles to a predefined input set. Let us discuss the algorithm below.

**LIDAR Reading**

LIDAR reading is taken ever 3 secs in the following directions with respect to the current heading angle: 0 deg, 30 deg, 90 deg, 270 deg, 330 deg.

![IMG_0135](https://github.com/JohirSuresh22/aue8230_sp24_team3/assets/158509706/405bf748-b28b-436e-8c85-df1831d7a684)

**The Predefined Input Set**

The input set consists of the following inputs,
| Array Index | 0 | 1 | 2 | 3 | 4 |
|-------------|---|---|---|---|---|
| Input Direction | Go Straight | Curving Right | Sharp Right | Sharp Left | Curving Left |
| Linear Velocity in x (m/s) | 0.2 | 0.2 | 0 | 0 | 0.2 |
| Angular Velocity in z (rad/s) | 0 | 1 | 1 | -1 | -1 |

**The Reward Function**

The reward function should ideally work with just lidar reading "d", but when we combine with rest of the code like wall-following, motion planning, april tag detection, etc, we would need weights for each of the functions. So for every lidar distance reading "d_i", the reward is calculated as follows,
J = R * d_i^2.
Here R is the weighting factor.

**The Algorithm**
- Every 3 secs, we get the LIDAR reading at the specified directions (augment maximum distance from 'inf' to '3.5').
- Calulate reward for each direction.
- Pick the direction with the highest reward and pick the corresponding input.

**Advantages**
- Very simple algorithm with O(3) computational complexity.
- A predefined input set elimites problems with a PID controller, such as integral windup, derivativies sensitivity towards noise, etc.
- In the biggest scheme of preparing for the project, this act as a small, computationally efficient part, that can be easily combined.
  
**Disadvantages**
- It is a very primitive controller and has discrete inputs instead of continuous ones. Although it works well for random motion, it needs to be combined with a motion planning algorithm to make it completely useful.
