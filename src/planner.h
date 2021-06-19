#include "json.hpp"
#include "spline.h"
#include <vector> 

#define N_LANES 3;
#define LANE_WIDTH 4;

using nlohmann::json;
using std::vector;

enum State {
  IDLE,
  CHANGING_LANE,
};

struct Costs{
  int vehicleId;
  double up;
  double back;
  double back_param1;
  double back_param2;
  double car_s;
  double prev_size;
  double time_step;
  double check_speed;
  double check_car_s;

  Costs() {
    up = 0;
    back = 0;
    back_param1 = 0;
    back_param2 = 0;
    car_s = 0;
    prev_size = 0;
    time_step = 0;
    check_speed = 0;
    check_car_s = 0;
  }
};

struct Trajectory {
  tk::spline spline;

  double cost;
  vector<Costs> costs;
  bool slow_down = false;
  int endLane;

  double ref_x;
  double ref_y;
  double ref_yaw;
};

class Planner {
  private: 
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Configuration paramters
    bool too_close = false;
    double target_vel = 49.5;
    int too_close_mt = 30;
    double time_step = 0.02;

    int prev_size;

    int lane;
    State state = IDLE;

    double ref_vel;
    double step_acc;

    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;

    // Previous path;
    double end_path_s;
    double end_path_d;

    nlohmann::json previous_path_x;
    nlohmann::json previous_path_y;

    // Sensor Fusion Data, a list of all other cars on the same side 
    //   of the road.
    nlohmann::json sensor_fusion;

    // Get possible trajectories
    void updateVelocity();

    Trajectory calculateTrajectory(int endLane);

    // Get the cost for trajectory
    double calculateCost(Trajectory &trajectory);

  public:
    Planner(vector<double> map_waypoints_x, 
      vector<double> map_waypoints_y, 
      vector<double> map_waypoints_s, 
      vector<double> map_waypoints_dx, 
      vector<double> map_waypoints_dy);
    // Previous;


    // This function read data from the telemetry and update info of the car 
    void updateTelemetry(nlohmann::json telemetry);

    // This function use sensor fusion data to calculate all possible trajectories
    vector<vector<double>> calculateTrajectory();


};