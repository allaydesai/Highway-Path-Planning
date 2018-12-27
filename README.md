# CarND-Path-Planning-Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Self-Driving Car Engineer Nanodegree Program

Overview

---

This project is a C++ implementation of a path planning module in a self driving car that creates smooth, safe trajectories for the car to follow. The module is used to guide the controller of a vehicle to move around a fixed map of waypoints. The map is off cars driving in a highway environment, all going different speeds, but approximately obeying the 50 MPH speed limit. Data pertaining to localization of the "ego" car and all other car's in the surrounding proximity is provided by the simulator. The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The car moves from point to point perfectly and every 20 ms the car moves to the next point on the list. The car's new rotation becomes the line between the previous waypoint and the car's new location. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration.

### Goals
- The car is able to drive at least 4.32 miles without incident.
- The car drives according to the speed limit of 50 MPH.
- Max Acceleration and Jerk are not exceeding 10 m/s^2 and 10 m/s^3 respectively.
- Car does not have collisions.
- The car stays in its lane, except for the time between changing lanes.
- The car is able to change lanes

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Data Format

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

* ["x"] The car's x position in map coordinates

* ["y"] The car's y position in map coordinates

* ["s"] The car's s position in frenet coordinates

* ["d"] The car's d position in frenet coordinates

* ["yaw"] The car's yaw angle in the map

* ["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

* ["previous_path_x"] The previous list of x points previously given to the simulator

* ["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

* ["end_path_s"] The previous list's last point's frenet s value

* ["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's 
* [0] car's unique ID

* [1] car's x position in map coordinates

* [2] car's y position in map coordinates

* [3] car's x velocity in m/s

* [4] car's y velocity in m/s

* [5] car's s position in frenet coordinates

* [6] car's d position in frenet coordinates 

---

## Path Planner

Role of the path planner module in a sefl driving car is to evaluate data computed from localization module and the sensor fusion module to generate potential trajectories which are evaluated in order to find the one that minimizes a set of cost functions that lead to an efficient and safe movement towards goal. 

The process of path planning is divided into 3 steps:
* Prediction: Predicts movement of surrounding objects 
* Behavior: Generates a plan considering the environments, predictions and goal.
* Trajectory: Generate efficient and safe trajectories 

### Implementation

The planner was implemented keepting project goals and constraints in mind. The goal was to continue driving ahead in a closed loop hence there is always a waypoint ahead. The prediction model is quite simple in this case as it is assumes that the surrounding vehicles do not change lanes often and mostly maintain a constant speed. Since the planner has a quick refresh rate it can react and adapt to changes quite easily allowing a constant speed and acceleration assumption. Finally the behavior planner is a simple state machine which comprises of 3 states, keep lane, change lane right, change lane left. 

All the planner coded lies within main.cpp which may not be the best organization but due to time constraints that was the path chosen. there are comments within the code which mark start and end of each module.

The approach consists of loading the car, previous path, end path and sensor fusion data from the simulator. We consider the fernet coordinates of the car which provides a s and d component that represents longitudinal and lateral displacement. According to the current state of the car, plan is generated taking into account traffic and speed. The search for surrounding cars is conducted using the s and d coordinates of surrounding cars provided in sesnor fusion data. We first find cars that are within the lane using the d component and then find the distance to the car if any in front using the s component. To account for delay in processing and maintain continuity, part of the previous trajectory is considered while generating new trajectory. 

Next lane change is considered if driving below desired velocity '''ref_vel''' and a car in front within some min gap distance. The lane change at present is not truly smart as it goes through the sequential steps of considering a left lane change first followed by a right lane change. It doesnot consider the distance (s) of cars in the target lane and pick lanes accordingly. Plus there is no cost constraints to emphasis passing cars from left lane according to traffic rules. 

Once the target lane is chosen the trajectory is generated using the help of spline library where we pick 3 anchor points at a distance of 30m, 60m and 90m these points have '''lane''' as a parameter hence fascilitating lane changing trajectory. Using the anchor points and the previous path points a new trajectory is generated with 50 path points. 

The velocity of the car is varied in the final step before loading the new path points in respective varaibles '''next_x_vals''' and '''next_y_vals'''. This is acheived by comparing the current '''car_speed''' with the '''ref_vel''' and then incrementing or decrementing the car speed by the velocity ramp. 

|        PARAMETERS           |VALUE| UNIT      |
|-----------------------------|-----|-----------|
|MAX_VELOCITY_IN_MPH          |49.5 | m         |
|GAP_REF_DISTANCE_IN_M        |30   | m         |
|MIN_GAP_DISTANCE_IN_M        |20   | m         |
|CYCLE_TIME_IN_S              |0.02 | 20ms      |
|M_PER_SEC_TO_MILES_PER_HOUR  |2.237| unit      |
|VELOCITY_RAMP                |0.224|5 m/sec^2  |
|NO_OF_POINT_PATH             |50   | unit      |

## Future Improvements 

There are several areas of improvement possible in future. I pointed out some while describing the planning process. Current organization of the code is not optimum and due to time constraints I was unable to split it into respective header files. Further currently I dont have cost functions for desired behavior. Adding cost functions that help follow traffic rules and selection of lanes better would help a great deal. When considering lane change currently I only consider the adjacent lane so if I am in lane 1, lane 3 would not be considered. Finally, Currently behavior planning and trajectory generation runs in the same time cycle which may not be the best choice. This planner is a basic implementation and there are various attributes that can be added to make it more robus and inclusive of real life scenarios. 

---
Getting Started
---
## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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
* A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

