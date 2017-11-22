#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

struct predictions  {
  bool current_lane_too_close = false;
  bool left_lane_open = true;
  bool right_lane_open = true;
};

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  double x;

  double y;

  double s;

  double d;

  double yaw;

  double speed;

  double lane;

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int goal_lane;

  int goal_s;

  string state;

  /**
  * Constructor
  */
  Vehicle(){}

  /**
  * Destructor
  */
  virtual ~Vehicle(){}

  string choose_next_state(predictions pred) {

    vector<string> states = successor_states();
    double cost;
    double best_cost = 999999999;
    string best_state;

    for(int i=0; i < states.size(); i++)
    {
      cost = calculate_total_cost(states[i], pred);
      if(cost < best_cost)
      {
          best_cost = cost;
          best_state = states[i];
      }
    }

    return best_state;
  }

  vector<string> successor_states() {
    /*
    Provides the possible next states given the current state for the FSM.
    Lane changes happen instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    if(state.compare("KL") == 0) {
        states.push_back("LCL");
        states.push_back("LCR");
    }
    return states;
  }

  double calculate_total_cost(string next_state, predictions pred)
  {

    //double comfort_cost = calculate_collision_cost(next_state, pred);
    //double legal_cost = calculate_legal_cost(next_state, pred);
    double collision_cost = calculate_collision_cost(next_state, pred);

    //double total_cost = 100 * collision_cost + 10 * legal_cost + 1 * comfort_cost;
    double total_cost = collision_cost;

    return total_cost;

  }

  double calculate_collision_cost(string next_state, predictions pred)
  {
    double collision_cost = 0;

    if(next_state == "KL" && pred.current_lane_too_close)
    {
      collision_cost = 0.5;
    }

    if(next_state == "LCL" && !pred.left_lane_open)
    {
      collision_cost = 1;
    }

    if(next_state == "LCR" && !pred.right_lane_open)
    {
      collision_cost = 1;
    }
  }

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<int> road_data);

};

#endif
