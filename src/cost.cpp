//
// Created by Sam on 8/6/2018.
//

#include <string>
#include <math.h>
#include "cost.h"

//static const double REACH_GOAL = pow(10, 1);
static const double EFFICIENCY = pow(10, 5);
static const double DENSITY = pow(10, 2);
static const double SPEED = pow(10, 1);
static const double GOAL_S = 6945.554;
static const double CHECKING_RANGE = 100;

static map<string, double> get_helper_data(const vector<Vehicle>& trajectory,
                                           map<int, Vehicle>& predictions);
//static double goal_distance_cost(map<string, double>& data);
static double inefficiency_cost(map<string, double>& data);
static double traffic_pool_cost(map<string, double>& data);
static double lane_speed_cost(map<string, double>& data);

double calculate_cost(const vector<Vehicle>& trajectory,
                      map<int, Vehicle>& predictions) {
  map<string, double> trajectory_data = get_helper_data(trajectory,
                                                        predictions);

  double cost(0.0);

  cost += (EFFICIENCY * inefficiency_cost(trajectory_data));
  cost += (DENSITY * traffic_pool_cost(trajectory_data));
  cost += (SPEED * lane_speed_cost(trajectory_data));

  return cost;
}


static map<string, double> get_helper_data(const vector<Vehicle>& trajectory,
                                           map<int, Vehicle>& predictions) {
  map<string, double> trajectory_data;
  Vehicle trajectory_start = trajectory[0];
  Vehicle trajectory_end = trajectory[1];

  double intended_lane;
  int state = trajectory_end.getCurrentState();

  if (state == Vehicle::STATE_PREP_LANE_CHANGE_LEFT) {
    intended_lane = trajectory_end.getCurrentLane() - 1;
  } else if (state == Vehicle::STATE_PREP_LANE_CHANGE_RIGHT) {
    intended_lane = trajectory_end.getCurrentLane() + 1;
  } else {
    intended_lane = trajectory_end.getCurrentLane();
  }

  auto final_lane = (double)trajectory_end.getCurrentLane();

  double distance_to_goal = GOAL_S - trajectory_end.getS();

  double vehicles_in_lane(0);
  double slowest_speed_in_lane(LEGAL_SPEED_LIMIT);
  Vehicle temp_vehicle;
  map<int, Vehicle>::const_iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;

    if (temp_vehicle.getCurrentLane() == intended_lane) {
      if ((temp_vehicle.getS() > trajectory_start.getS()) &&
          (temp_vehicle.getS() - trajectory_start.getS() <= CHECKING_RANGE)) {
        vehicles_in_lane += 1.0;

        double speed = temp_vehicle.getVelocity();
        if (slowest_speed_in_lane > speed) {
          slowest_speed_in_lane = speed;
        }
      }
    }
  }

  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
  trajectory_data["intended_speed"] = trajectory_end.getVelocity();
  trajectory_data["current_speed"] = trajectory_start.getVelocity();
  trajectory_data["vehicles_in_lane"] = vehicles_in_lane;
  trajectory_data["slowest_speed_in_lane"] = slowest_speed_in_lane;

  return trajectory_data;
}

//static double goal_distance_cost(map<string, double>& data) {
//  double cost;
//  double distance = data["distance_to_goal"];
//
//  if (distance > 0) {
//    cost = 1 - exp(-(fabs(data["intended_lane"] - data["final_lane"])) / distance);
//  } else {
//    cost = 1.0;
//  }
//
//  return cost;
//}

static double inefficiency_cost(map<string, double>& data) {
  return (data["current_speed"] - data["intended_speed"]);
}

static double traffic_pool_cost(map<string, double>& data) {
  return(1 - exp(-(data["vehicles_in_lane"])));
}

static double lane_speed_cost(map<string, double>& data) {
  return (data["slowest_speed_in_lane"] / LEGAL_SPEED_LIMIT);
}