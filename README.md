# Term3 Path Planning Project Writeup
Self-Driving Car Engineer Nanodegree Program

---

## Reflection


### 1. The goals of this project.
This project requires us to provide a path for our Ego vehicle that runs on the simulated highway. The path, speed and acceleration should take safety, legality and comfort in to consideration. We're given our Ego vehicle's location (both in Cartesian and Frenet coordinates), speed and yaw angle. In accordance with surrounding traffics feed by sensor fusion, we're to write a path planner that generates an optimal path, which let Ego vehicle drives as fast as possible at least 4.32 miles under 50MPH speed limit, without any incidents or jerks.

### 2. How I generate the path?
I first experimented the code given by Aaron in the `Project Walk Though` video. It turns out they've already been good enough for Ego vehicle to run smoothly in a single lane. And the lane changing curvature spits out by `spline.h` saves me a lot of work if I were to use Quintic Polynomial Solver. So, starting from this solid base, what remains for me to solve are i.) To provide a reference `velocity`. And ii.) To decide which `lane` the Ego vehicle should be running in, considering surrounding traffics. These two variables are exactly the problem in `Behavior Planning` lesson wants to tackle.

I designed a path planner almost the same as the final exercise does in `Behavior Planning` lesson. It consists of following steps:
  * Predict what surrounding traffics will be in the foreseeable future.

  I feed 50 path points to Ego vehicle in every actuation cycle, and it moves to next point after 20ms. The foreseeable future point is defined as 1 second ahead of the path ends in previous actuation cycle. This suggests I should choose the number of path points (time frame) meticulously. For if the points are too small, Ego vehicle ends up running out of path directives quickly. However, a large amount of points would give Ego too far away a future that couldn't keep up with ever changing environment.

  * Gather up all potential trajectories for Ego vehicle.

  The Ego vehicle has 6 states, defined in line 38 to 43 in `vehicle.h`, they are:
    0. INIT
    1. KEEP LANE
    2. PREPARE LANE CHANGE LEFT
    3. PREPARE LANE CHANGE RIGHT
    4. LANE CHANGE LEFT
    5. LANE CHANGE RIGHT

  For example, if Ego vehicle is in `KEEP LANE` state, the potential next states can only be `KEEP LANE`, `PREPARE LANE CHANGE LEFT` (if it's not in the leftmost lane), and `PREPARE LANE CHANGE RIGHT` (again, if it's not in the rightmost lane). Based on these next states, I generate a two point trajectory for each of them, starting from where Ego vehicle ends in previous path. Since I limited the maximum allowed acceleration to be 1 m/s^2 and the interval is 1 second, I simplify the kinematics that all vehicles move in a constant acceleration along Frenet coordinates `s`. In this way, I get the end point and velocity of the trajectory.

  *
