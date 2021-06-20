# Model documentation

To make a smooth trajectory I use the spline library as in Q&A introduction video.
There are 3 lanes so I use 0,1,2 to indexing the lanes. Each cycle I generate a trajectory using near lanes (for example if the car is in the lane 0 (left most lane) I calculate 2 possible trajectories one that keep lane and another one the change lane to the center lane.
After that I calculate a cost for each trajectory.
The cost considers vehicles in front of the car at the last time step of the trajectory that are in 60m ahead by increasing cost when are nearer, also the calculation includes vehicles that are in back and increase the cost as the the velocity of the vehicle is grater than the velocity of the car, this could make a lane change when starting because some vehicle that are came from the back are faster than the actual velocity.
