# Project Specification (Rubric)

## Compilation

### Criteria
The code compiles correctly.

### Speification
Code must compile without errors with cmake and make.

Given that we've made CMakeLists.txt as general as possible, it's recommend that you do not change it unless you can guarantee that your changes will still compile on any platform.

## Valid Trajectories

### Criteria
The car is able to drive at least 4.32 miles without incident..

### Speification
The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

### Criteria
The car drives according to the speed limit.

### Speification
The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

### Criteria
Max Acceleration and Jerk are not Exceeded.

### Speification
The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

### Criteria
Car does not have collisions.

### Speification
The car must not come into contact with any of the other cars on the road.

### Criteria
The car stays in its lane, except for the time between changing lanes.

### Speification
The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

### Criteria
The car is able to change lanes

### Speification
The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

## Reflection

### Criteria
There is a reflection on how to generate paths.

### Speification
The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".

## Suggestions to Make Your Project Stand Out!

Create a path planner that performs optimized lane changing, this means that the car only changes into a lane that improves its forward progress.

Incorporate a controller such as PID or MPC that follows the Path Planner's output path. Note that since the output path contains not only desired location information but also the car's desired speed as varying spaced points. One idea is to extract the desired speed from the path and then feed that into the controller. Another idea is if working with an MPC is to change the cost function so instead of evaluating cost relative to how close you are to a path, instead evaluate by how close the car is to one of the associating points of the path's output.
