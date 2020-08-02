## Path Planning
[![C++](https://img.shields.io/badge/C++-Solutions-blue.svg?style=flat&logo=c%2B%2B)](http://www.cplusplus.org/)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project we navigate through a virtual highway with other traffic, decissions for staying in the lane, change it, accelerate or deaccelerate are made in order to have a smooth trip. This project involves the Term 3 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

<img src="https://github.com/ajimenezjulio/P7_Path_Planning/blob/master/docs/path_planning.gif">
</p>

## Goal
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)

## Running the Code
A script for the process of cleaning, directory creation, build and execution was provided, so you only have to run it from the project root.
```
> ./clean_and_make.sh
```
For a manual approach the next commands should be executed.
```
> rm -r build
> mkdir build && cd build
> cmake .. && make
> ./path_planning
```

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

## Data Information

**Main car's localization Data (No Noise)**
- `["x"]` The car's x position in map coordinates
- `["y"]` The car's y position in map coordinates
- `["s"]` The car's s position in frenet coordinates
- `["d"]` The car's d position in frenet coordinates
- `["yaw"]` The car's yaw angle in the map
- `["speed"]` The car's speed in MPH

**Previous path data given to the Planner**
Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 
- `["previous_path_x"]` The previous list of x points previously given to the simulator
- `["previous_path_y"]` The previous list of y points previously given to the simulator

**Previous path's end s and d values**
- `["end_path_s"]` The previous list's last point's frenet s value
- `["end_path_d"]` The previous list's last point's frenet d value

**Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)**
- `["sensor_fusion"]` A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Implementation
The directory structure of this repository is as follows:

```
.
├── CMakeLists.txt
├── README.md
├── clean_and_make.sh
├── data
    ├── highway_map.csv  
├── src
    ├──  Eigen-3.3
        ├──  ...
    ├──  helpers.h
    ├──  path_planning.h
    ├──  spline.h
    ├──  json.hpp
    ├──  main.cpp
```

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

## Trajectory Generation Steps
**Planning**
1. We collect traffic data from the sensor fusion module of the simulator, processing them we activate flags that indicate the presence of cars on the left, right and in front of our car, in the same way we also calculate the speed of the car in front.
2. The next step is to obtain a list of possible paths given the current status of the car, each path consists of a tuple indicating the destination lane and the desired speed. 
3. Each path in the list is subsequently evaluated by a cost function to choose the one with the lowest cost. For this project, acceleration and speed were weighted higher than lane change.

**Trajectory Generation**
1. The first step is to choose 5 points that will form our path, the first two are taken from the last two points of the previous cycle or projecting the current position of the car. The next 3 take into consideration the destination lane and are taken at a uniform distance of 30 meters ahead each (30, 60, 90).
2. To ease calculations, the coordinate system is shifted in order to position the reference of the first point (origin) at 0 degrees from our car perspective.
3. Subsequently the 5 points are interpolated to fit using the spline function, then the curve is filled with a series of points that allows us to travel at the desired speed (30 points were chosen).
4. The trajectory is generated using 50 points taking those remaining from the last cycle and generating the missing points using the previously created and adjusted spline function.
5. Finally, the coordinate system is rotated again to return back to the original one (global coordinate system) and the trajectory is sent to the simulator.
