# Model documentation

To make a smooth trajectory I use the spline library as in Q&A introduction video.

## Assumptions

There are 3 lanes so I use 0,1,2 to indexing the lanes.

I decided to use 2 states:

- KEEP_LANE: when the car should stay in lane;
- CHANGING_LANE: when the car is moving to another lane, this state is locked since start changing lane to +/- 1mt from the center of the final lane (planner.cpp:200).

## Possible trajectories

In order to simplify the calculation of trajectories I decided to calculate the trajectory that follow the actual lane and trajectories that change to the near lanes. After that I calculate a cost for each trajectory (planner.cpp:204-219).

### Trajectory calculation

planner.cpp:79 `calculateTrajectory` function

To calculate the trajectory I followed the Q&A example by using two points of the previous path and adding 3 new points at 30-60-90 meters each using the actual velocity.

This are used to feed the spline instance and saved into a trajectory object.

## Cost function

planner.cpp:144 `calculateCost` function

The cost is evaluated using the final lane and at the last time step of the trajectory and considering the following points:

- The distance from the vehicles in front of the car that are in 60m ahead (planner.cpp:168-169): lower distance higher cost;
- The distance from vehicles behind 90m from the car considering the difference in velocity (planner.cpp:173-174). The cost increase when the velocity of the vehicle is grater the the velocity of the car.

## Final trajectory

The the final trajectory is choose by taking the lower cost trajectory and this trajectory make is a changing lane the state is set to CHANGING_LANE.
