# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Project Description and Setup

### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time.

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

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

# Write Up

## Overview

This project is a path planner that can be used with the Udacity simulator to autonomously drive a vehicle on a road with traffic. The vehicle follows map coordinates and will switch lanes in such a way that is maintains a speed near the limit of the highway.

## Modules

The path planner source code is located in the src/main.cpp file. The main function of the planner is broken into three distinct parts.
- Prediction
- Planning
- Trajectory Generation

### Prediction

The prediction module pulls in sensor fusion data from the Udacity simulator and checks the location of other vehicles on the road. The threshold value for this check is set to 25.0 meters. If a car is found in the current lane, an estimate of the car's location is made based on a simple dynamic model. The current speed of the car is estimated for the size of the previous path length. This estimates where the car will be in the future at the location of path completion.

```
// Prediction - Check for traffic in current lane
for (int i = 0; i < sensor_fusion.size(); i++)
{
  float d = sensor_fusion[i][6];

  if (d < (2+4*lane+2) && d > (2+4*lane-2))
  {
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt(vx*vx+vy*vy);
    double check_car_s = sensor_fusion[i][5];

    check_car_s += ((double)prev_size*0.02*check_speed);

    if((check_car_s > car_s) && ((check_car_s-car_s) < 25.0))
    {
      too_close = true;
    }
  }
}
```
If another car meets the criteria for being too close, a boolean value is flagged.

### Planning

The planning module takes the information provided from the prediction module and feeds that into a decision tree. The decision tree will decide which action (listed above) to take. This is done by flagging boolean values in the prediction module and feeding those values through the decision tree. If vehicles are identified in a proximity of < 25.0 meters, the primary vehicle will take one of the following finite actions:
- Stay in lane and slow down
- Change lanes to the right
- Change lanes to the left

If another vehicle is within 25.0 meters, the left lane is checked for other vehicles provided the primary vehicle is not in the left-most lane. Then the planner will check the right lane provided the vehicle is not in the right-most lane. If another vehicle is found, a false statement is flagged meaning that the lane is not clear to travel into. This is all done within a condition for the boolean flag created by the prediction module.
```
// Planner - Make descition if traffic is in current lane
if(too_close)
{

  // Check left lane
  if (lane > 0)
  {
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
      float s = sensor_fusion[i][5];
      float d = sensor_fusion[i][6];

      if (d < (-2+4*lane+2) && d > (-2+4*lane-2) && fabs(s-car_s) < 15.0)
      {
        clear_l = false;
      }
    }
  }
```

Lane checking is accomplished by meeting the criteria for being in the lane (to the left in the code above) and being within +/- 15.0 meters from the current car. This is a crude assessment and can be further improved by taking into consideration the future states of the vehicles in other lanes. This method tends to work in most cases though and is computationally simple.

Finally, the boolean values are passed through a conditional tree with priority for moving to the right lane.
```
// Make a decision of where to go
if (clear_r)
{
  lane = lane+1;
}
else if (clear_l)
{
  lane = lane-1;
}
else
{
  ref_vel -= 0.385;
}
```

If both the right lane and left lane are not clear, the vehicles velocity will be reduced. If no vehicles are identified in the primary vehicles path, the vehicle will accelerate at a rate no to exceed the limits of the project rubric. This is done to maintain passenger comfort. The conditional test used to determine if vehicles were present is shown below:

### Trajectory Generation

The trajectory generation module takes the lane information from the planning module and creates a set of waypoints for the vehicle to move too. To smooth the trajectory, the last two values from the previous path and three new waypoints set at 30m, 60m, and 90m are placed into vectors. Those waypoints are then fed into a spline generation tool.

## Results

During test, the planner performed adequately well with respect to the project rubric. Initial cases were able to pass the 4.32 mile mark with no incidents. However, during subsequent tests it was seen that erratic behavior from other cars on the road tended to yield collisions.

![test_435](https://github.com/mblomquist/Udacity-SDCND-Term3/blob/master/P1/pictures/test_4_35.JPG)

The performance of the model was improved by adjusting the threshold values used in the conditional statements. For example, a 25.0 meter threshold was used to determine if action needed to be taken given a car in the lane ahead.

![test_510](https://github.com/mblomquist/Udacity-SDCND-Term3/blob/master/P1/pictures/test_5_10.JPG)

By changing the threshold for checking the left and right lanes to 15.0 meters, quicker manuvers were able to be taken and improve the average speed. In the best case, a range of 7.61 miles was achieved. 

![test_705](https://github.com/mblomquist/Udacity-SDCND-Term3/blob/master/P1/pictures/test_7_05.JPG)
![test_761](https://github.com/mblomquist/Udacity-SDCND-Term3/blob/master/P1/pictures/test_7_61.JPG)

## Additional Work

After reviewing the project code, I think additional work can be spent on the planning module. The simplistic decision tree works for this particular project, but would need to be enhanced for implementation in a real world senario. Additionally, the future state of cars in other lanes could be incorporated into the planner to get a better assessment of road condition in the future.
