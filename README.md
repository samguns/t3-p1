# Term3 Path Planning Project Writeup
Self-Driving Car Engineer Nanodegree Program

---

## Reflection


### 1. The goals of this project.
This project requires us to provide a path for our Ego vehicle that runs on a simulated highway. The path, speed and acceleration should take safety, legality and comfort into consideration. We're given our Ego vehicle's location (both in Cartesian and Frenet coordinates), speed and yaw angle. In accordance with surrounding traffics feed by sensor fusion, we're to write a path planner that generates an optimal path, which let Ego vehicle drives as fast as possible at least 4.32 miles under 50MPH speed limit, without any incidents or jerks.

### 2. How I generate the path?
I first experimented the code given by Aaron in the `Project Walk Though` video. It turns out they've already been good enough for Ego vehicle to run smoothly in a single lane. And the lane changing curvature spits out by `spline.h` saves me a lot of work if I were to use Quintic Polynomial Solver. So, starting from this solid base, what remains are i.) To provide a `velocity`. And ii.) To decide which `lane` the Ego vehicle should be running in, considering surrounding traffics. These two variables are exactly the problem in `Behavior Planning` lesson wants to tackle.

My path planner borrowed as many code from the final exercise in `Behavior Planning` lesson as possible. It consists of following steps:
  * Predict what other vehicles will be in the near future.

  The predicting future period starts from the end point of previous actuation cycle. If there's no previous actuation left, like a cold start, or just runs out of path points, then the starting point is where we are now.
  Because the road in simulator is a circle of about 6945.554 meters, the `s` coordinates turns to 0 when the vehicle finishes a round. This is inconvenient to calculate safe distances . So I made a workaround that i.) adding a 6945.554 to `s` coordinates of those vehicles, which completes a round, if Ego vehicle is less than 30 meters behind the finish line. ii.) Subtracting a 6945.554 for those that haven’t finished, if Ego is less than 20 meters ahead of finish line.

  * Gather up all potential trajectories for Ego vehicle.

  By comparing the predicted information with Ego vehicle's position and velocity, I generate a two point trajectory for each potential next states (among all states of FSM). Here's some notes I’d like to point out.
	 * The trajectory tells us in which lane and at what speed it should be for the one-second long future period.
	* To avoid an uncomfortable changing acceleration rate (known as jerk), I limited the maximum acceleration of Ego vehicle to be 0.1 m/s^2, and set a 25-meters safe distance buffer. So that when it comes to braking, a relatively slower deceleration won’t make our Ego vehicle bumps into cars ahead.
	* In `prepare lane change left/right` state, not only other cars in the lane next to us should be checked, but those in next to that lane, if it’s available, should be considered as well. For example, in our 3-lane highway, we are in the leftmost lane, namingly 0. We should check traffics not only in lane 1, but also in lane 2. Because if we think it’s possible for us to change from lane 0 to lane 1, and if another car in lane 2 holds this same idea that it can turn into lane 1, and as it happens, we’re almost running side by side. A collision is nearly inevitable.
	* There exists some chance that right after a lane change has been realized, it’s possible to do another attempts. To avoid such seemingly consecutive lane changing behaviors, I suppressed it by adding a timer that only if two seconds elapsed, could another lane changing maneuver be considered.

* Calculate cost of each trajectory and choose the lowest.

  I came up with three simple cost functions in deciding which trajectory is the best. They are,
	* `inefficiency_cost`, which has the highest weight, penalizes a deceleration maneuver.
	* `traffic_pool_cost`, which checks the density of a lane 100 meters in front that we’re about to turn into. It helps in choosing a preferable lane that prevents us from surrounded by slower vehicles around.
	* The lowest weight cost function `lane_speed_cost` penalizes the lane that we intend to change into has a relatively slower car, when we’re able to change to left lane or right lane.

### 3. Discusses improvements.
In every processing cycle, I think it might be possible to maintain a belief table for every vehicle feeds from sensor fusion. That is, for a given vehicle, maintains its `keep straight`, `change lane left` and `change lane right` belief in a Gaussian distribution. I believe such belief possibilities could help in generating a better next maneuver.
The Ego vehicle seems working well by now, but sometimes it still ran into a lane that eventually got surrounded with a slower speed. I think if we slow down first, and change to a sparse lane earlier just like a human does, could be a solution to avoid being trapped. This is a thinking out loud idea though, for it suggests a rather complicated decision making design I’m not sure if it’s worth to explore further yet.
