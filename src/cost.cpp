//
// Created by Sam on 8/6/2018.
//

#include <string>
#include <math.h>
#include "cost.h"

static map<string, double> get_helper_data(const vector<Vehicle>& trajectory);
static double goal_distance_cost(map<string, double>& trajectory_data);

double calculate_cost(const vector<Vehicle>& trajectory) {
  map<string, double> trajectory_data = get_helper_data(trajectory);

  double cost = 0.0;

  cost += goal_distance_cost(trajectory_data);

  return cost;
}


static map<string, double> get_helper_data(const vector<Vehicle>& trajectory) {
  map<string, double> trajectory_data;
  Vehicle trajectory_start = trajectory[0];
  Vehicle trajectory_end = trajectory[1];

  double intended_lane = -1;
  int state = trajectory_end.getCurrentState();

  if (state == Vehicle::STATE_PREP_LANE_CHANGE_LEFT ||
      state == Vehicle::STATE_PREP_LANE_CHANGE_RIGHT ||
      state == Vehicle::STATE_LANE_CHANGE_LEFT ||
      state == Vehicle::STATE_LANE_CHANGE_RIGHT) {
    intended_lane = (double)trajectory_start.getCurrentLane();
  }

  auto finale_lane = (double)trajectory_end.getCurrentLane();

  double distance_to_goal = trajectory_end.getS() - trajectory_start.getS();

  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = finale_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;

  return trajectory_data;
}

static double goal_distance_cost(map<string, double>& data) {
  double cost;
  double distance = data["distance_to_goal"];

  if (distance > 0) {
    cost = 1 - exp(-(fabs(data["intended_lane"] - data["final_lane"])) / distance);
  } else {
    cost = 1.0;
  }

  return cost;
}