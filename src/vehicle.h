#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

// Container to hold the predictions
struct predictions  {
  bool current_lane_too_close = false;
  bool left_lane_open = true;
  bool right_lane_open = true;
};

// Safe distance to keep from vehicle ahead in ego's lane
static double CURRENT_LANE_VEHICLE_AHEAD_SAFE_DISTANCE = 30.0;

// Safe distance to keep from vehicle ahead in target lane when planning for a lane change
static double TARGET_LANE_VEHICLE_AHEAD_SAFE_DISTANCE = 40.0;

// Safe distance to keep from vehicle behind in target lane when planning for a lane change
static double TARGET_LANE_VEHICLE_BEHIND_SAFE_DISTANCE = 15.0;

class Vehicle {
public:

  double x;

  double y;

  double s;

  double d;

  double yaw;

  double speed;

  int lane;

  string state;

  /**
  * Constructor
  */
  Vehicle(){}

  /**
  * Destructor
  */
  virtual ~Vehicle(){}

  /**
  * Function that returns the lane based on the d value passed to it
  */
  static int get_lane(int d){
    int lane;
    if(d < 4)
    {
      lane = 0;
    }
    else if(d > 8)
    {
      lane = 2;
    }
    else
    {
      lane = 1;
    }
    return lane;
  }

  /**
  * Function that determines the next best state for ego based on predictions passed to it
  */
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

  /**
  * Function that determines the next available states for ego based on its current state
  */
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

  /**
  * Function that computes the total cost for a maneuver given the current state and predictions
  */
  double calculate_total_cost(string next_state, predictions pred)
  {

    double safety_cost = calculate_safety_cost(next_state, pred);
    double legal_cost = calculate_legal_cost(next_state, pred);
    double comfort_cost = calculate_comfort_cost(next_state, pred);

    double total_cost = 100 * safety_cost + 10 * legal_cost + 1 * comfort_cost;

    return total_cost;

  }

  /**
  * Function that computes the safety cost for a maneuver given the current state and predictions
  */
  double calculate_safety_cost(string next_state, predictions pred)
  {
    double safety_cost = 0;

    // Increase cost is there is a vehicles ahead in the same lane within safe distance
    if(next_state == "KL" && pred.current_lane_too_close)
    {
      safety_cost = 0.05;
    }

    // Increase cost for left lane change if there are vehicles ahead or behind in the left lane within safe distance
    if(next_state == "LCL" && !pred.left_lane_open)
    {
      safety_cost = 1;
    }

    // Increase cost for right lane change if there are vehicles ahead or behind in the right lane within safe distance
    if(next_state == "LCR" && !pred.right_lane_open)
    {
      safety_cost = 1;
    }

    return safety_cost;
  }

  /**
  * Function that computes the legal cost for a maneuver given the current state and predictions
  */
  double calculate_legal_cost(string next_state, predictions pred)
  {
    double legal_cost = 0;

    if(next_state == "KL")
    {
      legal_cost = 0;
    }

    // Increase cost for left lane change if ego is already in the leftmost lane
    if(next_state == "LCL" && lane == 0)
    {
      legal_cost = 1;
    }

    // Increase cost for right lane change if ego is already in the rightmost lane
    if(next_state == "LCR" && lane == 2)
    {
      legal_cost = 1;
    }

    return legal_cost;
  }

  /**
  * Function that computes the comfort cost for a maneuver given the current state and predictions
  */
  double calculate_comfort_cost(string next_state, predictions pred)
  {
    double comfort_cost = 0;

    // Lowest cost for staying in current lane as it is the most comfortable
    if(next_state == "KL")
    {
      comfort_cost = 0;
    }

    // Increase cost for any lane change
    if(next_state == "LCL" or next_state == "LCR")
    {
      comfort_cost = 1;
    }

    return comfort_cost;
  }

};

#endif
