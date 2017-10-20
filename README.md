[//]: # (Image References)

[tailGate]: ./img/tailGate_big.gif "Rear-ended"

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   

### Goals
In this project, the goal is to safely navigate around a virtual highway with other traffic that is driving `+-10 MPH`
of the `50 MPH speed limit`. The simulator provides the vehicle's localization and sensor fusion data; there is also a
sparse map list of waypoints around the highway. The vehicle (from now on referred to as the **Ego**) should try to go
as close as possible to the `50 MPH speed limit`, which means passing slower traffic when possible, while other cars 
(from now on referred to as the **Bees**) may change lanes too.
The **Ego** should avoid colliding with the **bees** at all cost as well as driving inside of the marked road lanes at all times unless going from one lane to another. The Ego should be able to make one complete loop around the `6946 m` highway loop. Since it is trying to go `50 MPH`, it should take a little over `5` minutes to complete one loop. Also, the Ego should not experience total acceleration over `10` m/s^2 and jerk that is greater than `10` m/s^3.

#### Map
 
The map data of the highway is stored in `data/highway_map.txt`.
Each waypoint in the list contains `[x,y,s,dx,dy]` values. `x` and `y` are the waypoint's **Cartesian** coordinates on 
the map; the `s` value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, a distance along the road, goes from `0` to `6945.554`.

The power of [Spline](http://kluge.in-chemnitz.de/opensource/spline/) functionality has been heavily leveraged throughout the project.

Among the 5 elements of the waypoint data, `s` is guaranteed not to have duplicate values. Thus, 4 `tk::spline` objects 
have been created for `x`, `y`, `dx` and `dy` with respect to `s`. Those splines are used by the instance of 
the `PathAgent` class to obtain the **Cartesian** (`{x, y}`) coordinates from the **Frenet** (`{s, d}`).

#### Algorithm

1. Obtain current Ego telemetry, sensor fusion data, and information of the remainder of the previous path.
2. Preserve the state of the Ego at the end of the previous path. It is being used as a starting point for computing
the path extension. 
3. Store the forward proximity for each of three lanes in a `vector` of structured `Proxy` states.
4. Evaluate the ***attractiveness*** of each lane depending on its free forward space and remoteness from the current lane.
5. Recursively evaluate lanes for safety (absence of collisions), starting from the most attractive to the least attractive
until the safe lane is found. If no reliable lane determined other than the current lane, the latter is
picked and its safety being enforced by following the `proxy` at a safe distance.
6. Generate new path, attach it to the previously unused path and send it to the Simulator.

`StateAgent` class is responsible for points 1 - 3, while `PathAgent` performs 4 - 6. 

#### Planning

In my opinion, the task at hand is an excellent fit for [Q-Learning](https://en.wikipedia.org/wiki/Q-learning),
with the perfect coverage of [our domain-specific implementation](https://youtu.be/QDzM8r3WgBw) by 
[Lex Fridman](https://www.youtube.com/user/lexfridman/videos). But, apart from the colossal pain it takes to make **TensorFlow**
work in C++ (while it is itself written in C++), the Simulator interface makes the task almost prohibitively hard.

I have thoroughly studied the **Final State Machine** approach covered in classes.
To say the least, this is not my style of decision-making.

**The algorithm that I've finally derived continually scans through the available lanes and picks the most attractive among those that are collision-free.**

#### Trajectory

For trajectory, I have extensively borrowed the techniques used by [Aaron Brown](https://github.com/awbrown90) 
in [his native implementation](https://github.com/awbrown90/CarND-Path-Planning-Project). In fact, I **do** have a successful implementation using the concept of **Jerk Minimizing Trajectories** provided in classes but failed to prevent it from **breaking the jerk/acceleration** limits in some marginal states.
Aaron is presumably the co-author of the Simulator and its interface with the outer world, so he probably knows the best way to interact with it.

#### Results

The implementation more or less guarantees that no incidents will be triggered by the Ego. It also processes the situations when **Bee** unexpectedly swerves into the Ego's lane right in front of it.

However, situations like the one below **aren't** processed.

![alt text][tailGate]

This looks like a [common self-driving car problem](https://sf.curbed.com/2017/6/12/15781292/self-driving-car-accidents) when human drivers rear-end autonomous vehicles  and is beyond the scope of the [Projects rubric](https://review.udacity.com/#!/rubrics/1020/view), although it obviously possible to properly address such situations.

The sample video of successfully passing the entire highway loop may be found on [my humble YouTube channel](https://youtu.be/BV0rfsj1ALw).

### Basic Build Instructions

I've added two new sources to `CMakeLists.txt`: `src/stateAgent.cpp` and `src/pathAgent.cpp`

1. `mkdir build && cd build`
2. `cmake .. && make`
3. `./path_planning`.

