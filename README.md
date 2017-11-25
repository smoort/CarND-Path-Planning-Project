# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Reflection on Path Generation
Below is the approach followed for generating the trajectory

1. The target lane is computed through the behaviour planning module by passing the predictions.
1. 5 points are generated from which a spline can be created.  The points chosen are shown below :
*  The current position of ego is the second point.
*  The first point is taken from the previous path returned by the simulator.  If the previous path does not exist, the first point is extrapolated using the current position and yaw of ego.
*  The third, fourth and fifth points are calculated using ego's current position, target lane and at a distance of 30m, 60m and 90m respectively.
3. The 5 points are then shifted to car coordinates to keep the math and intution simple.
1. A spline is generated using the 5 shifted points.
1. Points available in the previous path are added to the planner first.
1. Y value at a target x distance of 30m in front of car is calculated
1. The target distance is calculated as the hypotenues of x and y
1. The target distance is broken down into small chunks based on the number of point that are still needed for the planner
1. For each chunk, the distance moved is calculated using the target distance, time lapsed and reference velocity adjusted for mph to meters per second
1. The x and y values for the new location is calculated using the current x,y and yaw values.
1. Once all the points for the path are filled, the path is sent to the simulator for execution.
