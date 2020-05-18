# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---
## Introduction

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Car's localization and sensor fusion data will be provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

To justify these objectives, I developed a the path planning algorithm as follows:

1. Sparse map waypoints interpolation
2. Planner initialization
3. Nearby obstacle information assignment to each planner
4. Jerk-minimizing trajectory generation
5. Optimal trajectory selection

## Implementation
### Sparse map waypoints interpolation

Given simulator map is recorded at 30m intervals, so smooth trajectory cannot be obtained directly using `Cartesian` function. Hence waypoints are interpolated using `spline` function with 0.1m interval. With help of these coarse waypoints we get smoothe trajectory in global coordniates.

### Planner initialization

Driving is fairly easy and structural on highways. Planner is assigned for each lane, and each planner creates best trajectory fro that particular lane.Each planner contains following elements:

```c++
typedef struct Planner {
  double target_d;
  vector<Vehicle> obstacles;
  Vehicle target_to_follow;
  int following_target_id;
  double dist_to_target;
  MatrixXd s_trajectories;
  VectorXd s_costs;
  MatrixXd d_trajectories;
  VectorXd d_costs;
  bool obstacle_following;
  bool feasible_traj_exist;
  int optimal_s_id;
  int optimal_d_id;
  double minimal_cost;
  int iters;
} Planner;
```

### Nearby obstacle information assignment to each planner

The position and velocity information of nearby vehicles from `sensor_fusion` is transmitted to each planner. If there is obstacle in the lane, car will  go into `obstacle_following` or if there is nothing in the lane car will go into `velocity_keeping`.

### Jerk-minimizing trajectory generation

`obstacle_following` and `velocity_keeping` trajectories are generated without considering collision. Here **5th order polynomial** trajectories are created that minimizes jerk in Frenet frame independently fro s and d. Costs considered are:

1. Jerk (smaller is favorable)
2. Terminal state (safe distance in s-direction and cross track error in d-direction)
3. Speed (near to `max_speed` is favorable)

After this jerk is minimized and speed is kept near maximum speed to keep the cost low. [Huber Loss](https://en.wikipedia.org/wiki/Huber_loss) is used to speed up the speed up to` max_speed`. Total cost in calculated considering longitudinal and lateral costs.

```c++
sd_cost(ss,dd) = klon * planners[i].s_cost[ss] + klat * planners[i].d_cost[dd];
```

### Optimal trajectory selection

Path planner selects collision free trajectory in Frenet frame with lowest cost of each planner.

1. Best collision free trajectory is calculated for each planner
2. Optimal trajectory  with lowest cost amongst is selected

## Conclusion

It is fairly possible to drive withput difficulty in all traffic consditions and when there is a safe path available with speed close to `max_speed`, ego vehicle changes lane. 

# Udacity README   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

