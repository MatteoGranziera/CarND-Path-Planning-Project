#include "json.hpp"
#include "planner.h"
#include "helpers.h"

using nlohmann::json;
using std::cout;
using std::endl;

Planner::Planner(vector<double> map_waypoints_x, 
      vector<double> map_waypoints_y, 
      vector<double> map_waypoints_s, 
      vector<double> map_waypoints_dx, 
      vector<double> map_waypoints_dy){

  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_dx = map_waypoints_dx;
  this->map_waypoints_dy = map_waypoints_dy;

  lane = 1;
  ref_vel = 0.0;
  step_acc = 0.224;
}

void Planner::updateTelemetry(nlohmann::json telemetry){
  car_x = telemetry[1]["x"];
  car_y = telemetry[1]["y"];
  car_s = telemetry[1]["s"];
  car_d = telemetry[1]["d"];
  car_yaw = telemetry[1]["yaw"];
  car_speed = telemetry[1]["speed"];

  // Previous path data given to the Planner
  previous_path_x = telemetry[1]["previous_path_x"];
  previous_path_y = telemetry[1]["previous_path_y"];

  // Previous path's end s and d values 
  end_path_s = telemetry[1]["end_path_s"];
  end_path_d = telemetry[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side 
  //   of the road.
  sensor_fusion = telemetry[1]["sensor_fusion"];

  prev_size = previous_path_x.size();

  if(prev_size > 0 ){
    car_s = end_path_s;
  }
}

void Planner::updateVelocity(){

  target_vel = 49.5;

  for(int i = 0; i < sensor_fusion.size(); i++){
    float d = sensor_fusion[i][6];

    if(d < (2+4*lane+2) && d > (2+4*lane-2)){
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];

      double check_speed =  sqrt(vx*vx+vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s+= ((double)prev_size*time_step*check_speed);

      if(check_car_s > car_s && (check_car_s-car_s) < too_close_mt){
        // cout << sensor_fusion[i][0] << " - check speed: " << check_speed << endl;

        target_vel = check_speed*2.24 - (1/exp(check_car_s-car_s) * 5) ;
        // cout<< "Update Velocity: " << target_vel<< endl;
      }
    }
  }
}

Trajectory Planner::calculateTrajectory(int endLane){

  Trajectory trajectory;
  trajectory.endLane = endLane;

  vector<double> ptsx;
  vector<double> ptsy;

  trajectory.ref_x = car_x;
  trajectory.ref_y = car_y;
  trajectory.ref_yaw = deg2rad(car_yaw);

  // Initialize previous pints
  if(prev_size < 2){
      ptsx.push_back(car_x - cos(car_yaw));
      ptsx.push_back(car_x);

      ptsy.push_back(car_y - sin(car_yaw));
      ptsy.push_back(car_y);
  } else {
      trajectory.ref_x = previous_path_x[prev_size-1];
      trajectory.ref_y = previous_path_y[prev_size-1];

      double ref_x_prev = previous_path_x[prev_size-2];
      double ref_y_prev = previous_path_y[prev_size-2];
      trajectory.ref_yaw = atan2(trajectory.ref_y-ref_y_prev, trajectory.ref_x-ref_x_prev);

      ptsx.push_back(ref_x_prev);
      ptsx.push_back(trajectory.ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(trajectory.ref_y);
  }

  // Add 3 more points
  vector<double> next_wp0 = getXY(car_s+30, (2+4*endLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, (2+4*endLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, (2+4*endLane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // Shift car reference to 0 degrees
  for(int i=0; i< ptsx.size(); i++){
    double shift_x = ptsx[i] - trajectory.ref_x;
    double shift_y = ptsy[i] - trajectory.ref_y;

    ptsx[i] = (shift_x * cos(0-trajectory.ref_yaw) - shift_y * sin(0-trajectory.ref_yaw));
    ptsy[i] = (shift_x * sin(0-trajectory.ref_yaw) + shift_y * cos(0-trajectory.ref_yaw));

  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  trajectory.spline = s;

  return trajectory;
}

double Planner::calculateCost(Trajectory &trajectory){
  double cost = 0.0;

  // For each vehicle in sensor fusion data
  for(int i = 0; i < sensor_fusion.size(); i++){
    float d = sensor_fusion[i][6];

    // if is in the same future lane
    if(d < (2+4*trajectory.endLane+2) && d > (2+4*trajectory.endLane-2)){
      Costs costs; 
      costs.vehicleId = sensor_fusion[i][0];

      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];

      double check_speed =  sqrt((vx*vx)+(vy*vy));
      double check_car_s = sensor_fusion[i][5];

      // Calculate future position of the vehicle
      check_car_s+= ((double)prev_size*time_step*check_speed);

      double ref_vel_mt_s = ref_vel/2.24;

      // Check if the vehicle is in a valuable up range for the car to care about
      if(check_car_s > car_s && (check_car_s-car_s) < 60){
        cost+= pow( 30 / (check_car_s-car_s) , 2);
      }

      // Check if the vehicle is in a valuable back range for the car to care about
      if(check_car_s < car_s && (car_s-check_car_s) < 90 + ((check_speed - ref_vel_mt_s))){
        cost+= pow(((20 + (check_speed - ref_vel_mt_s))) / (car_s-check_car_s), 3);

        
      }
      trajectory.costs.push_back(costs);
    }

  }

  return cost;
}

vector<vector<double>> Planner::calculateTrajectory(){
  updateVelocity();

  if(ref_vel > target_vel)
  {
    ref_vel-=.433 - (1/exp((ref_vel-target_vel))*.433);
    // cout << "target: " << target_vel << " - ref_vel:" << ref_vel << endl;
  } 
  else if(ref_vel < target_vel)
  {
    ref_vel+=.433;
  }

  // If the car is in the correct choosen lane reset to KEEP_LANE state  +- 0.5
  if(state == CHANGING_LANE && car_d < (2+4*lane+1) && car_d > (2+4*lane-1)){
    state = KEEP_LANE;
  }

  vector<Trajectory> trajectories;

  Trajectory keepTrajectory = calculateTrajectory(lane);
  keepTrajectory.cost = calculateCost(keepTrajectory);

  if(lane + 1 < 3){
    Trajectory changeRightLane = calculateTrajectory(lane+1);
    changeRightLane.cost = calculateCost(changeRightLane);
    trajectories.push_back(changeRightLane);
  }

  if(lane - 1 >= 0){
    Trajectory changeLeftLane = calculateTrajectory(lane-1);
    changeLeftLane.cost = calculateCost(changeLeftLane);
    trajectories.push_back(changeLeftLane);
  }

  Trajectory trj = keepTrajectory;

  if(state == KEEP_LANE)
  {
    for(int i = 0; i < trajectories.size(); i++){
      if(trajectories[i].cost < trj.cost){
        trj = trajectories[i];

        state = CHANGING_LANE;
        lane = trj.endLane;
      }
    }
  }

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Use previous points
  for(int i = 0; i < previous_path_x.size(); i++){
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = trj.spline(target_x);
  double target_dist = sqrt((target_x*target_x) + (target_y*target_y));

  double x_add_on = 0;

  for(int i = 0; i < 50-previous_path_x.size(); i++){
    double N = (target_dist/(0.02*ref_vel/2.24));
    double x_point = x_add_on + (target_x/N);
    double y_point = trj.spline(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Shift back to the original angle
    x_point = (x_ref *cos(trj.ref_yaw) - y_ref*sin(trj.ref_yaw));
    y_point = (x_ref *sin(trj.ref_yaw) + y_ref*cos(trj.ref_yaw));

    x_point += trj.ref_x;
    y_point += trj.ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  
  vector<vector<double>> next_vals;
  next_vals.push_back(next_x_vals);
  next_vals.push_back(next_y_vals);
  return next_vals;
}




