# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Reflection on Path Generation
Below is the approach followed for generating the trajectory

1. The target lane is computed through the behaviour planning module by passing the predictions.
1. 5 points are generated from which a spline can be created.  The points chosen are shown below :
   a.  The current position of ego is the second point.
   b.  The first point is taken from the previous path returned by the simulator.  If the previous path does not exist, the first point is extrapolated using the current position and yaw of ego.
   c.  The third, fourth and fifth points are calculated using ego's current position, target lane and at a distance of 30m, 60m and 90m respectively.
1. The 5 points are then shifted to car coordinates to keep the math and intution simple.
1. A spline is generated using the 5 shifted points.
1. Points available in the previous path are added to the planner first.
1. 

