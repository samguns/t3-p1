#include <utility>

//
// Created by Sam on 2018/8/4.
//

#include <iostream>
#include <math.h>
#include <algorithm>
#include "vehicle.h"
#include "cost.h"


Vehicle::Vehicle() :
  mCurrentState(STATE_INIT)
{}

Vehicle::Vehicle(vector<double> sensored_state) :
  mS(sensored_state[SENSOR_FUSION_S_IDX]),
  mD(sensored_state[SENSOR_FUSION_D_IDX]) {
  double vx = sensored_state[SENSOR_FUSION_VX_IDX];
  double vy = sensored_state[SENSOR_FUSION_VY_IDX];

  mVelocity = sqrt(vx*vx + vy*vy);

  for (int lane = 0; lane < MAX_NOF_LANES; lane++) {
    if (mD < (LANE_HALF_WIDTH + LANE_WIDTH * lane + LANE_HALF_WIDTH) &&
        mD > (LANE_HALF_WIDTH + LANE_WIDTH * lane - LANE_HALF_WIDTH)) {
      mCurrentLane = lane;
    }
  }
}

Vehicle::Vehicle(int lane, double s, double v, int state) :
  mCurrentLane(lane), mS(s), mVelocity(v), mCurrentState(state) {}

void Vehicle::configure(double max_speed, int currentLane, vector<int> lanes) {
  mMaxSpeedLimit = max_speed;
  mNumberOfLanes = lanes.size();
  mLeftMostLane = lanes[0];
  mRightMostLane = lanes[mNumberOfLanes-1];
  mCurrentLane = currentLane;
}

void Vehicle::update(double s, double v) {
  mS = s;
  mVelocity = v;
}

void Vehicle::getNextBehavior(int prev_size, vector<vector<double>> sensor_fusion,
                              int &goal_lane, double &goal_v) {
  map<int, Vehicle> predictions;

  /* Predict other vehicles' location */
  for (const auto &sensor_state : sensor_fusion) {
    Vehicle pred = Vehicle(sensor_state);
    pred.generate_prediction(prev_size);

    predictions[sensor_state[SENSOR_FUSION_ID_IDX]] = pred;
  }

  /* Iterate all possible behaviors for ego vehicle.
   * Calculate cost for every behavior.
   */
  double cost;
  vector<double> costs;
  vector<vector<Vehicle>> final_trajectories;
  vector<int> states = successor_states();
  for (const int& state : states) {
    vector<Vehicle> trajectory = generate_trajectory(state, predictions);
    if (!trajectory.empty()) {
      cost = calculate_cost(trajectory);
      costs.push_back(cost);
      final_trajectories.push_back(trajectory);
    }
  }

  vector<double>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = distance(begin(costs), best_cost);

  vector<Vehicle> best_trajectory = final_trajectories[best_idx];
  this->mCurrentState = best_trajectory[1].getCurrentState();
  this->mCurrentLane = best_trajectory[1].getCurrentLane();
  this->mVelocity = best_trajectory[1].getVelocity();

  goal_lane = this->mCurrentLane;
  goal_v = this->mVelocity;
}

int Vehicle::getCurrentState() {
  return mCurrentState;
}

int Vehicle::getCurrentLane() {
  return mCurrentLane;
}

double Vehicle::getS() {
  return mS;
}

double Vehicle::getVelocity() {
  return mVelocity;
}


vector<int> Vehicle::successor_states() {
  vector<int> states;

  states.push_back(STATE_KEEP_LANE);

  switch (mCurrentState) {
    case STATE_KEEP_LANE:
      if (mCurrentLane != mLeftMostLane) {
        states.push_back(STATE_PREP_LANE_CHANGE_LEFT);
      }

      if (mCurrentLane != mRightMostLane) {
        states.push_back(STATE_PREP_LANE_CHANGE_RIGHT);
      }

      break;

    case STATE_PREP_LANE_CHANGE_LEFT:
      if (mCurrentLane != mLeftMostLane) {
        states.push_back(STATE_PREP_LANE_CHANGE_LEFT);
        states.push_back(STATE_LANE_CHANGE_LEFT);
      }
      break;

    case STATE_PREP_LANE_CHANGE_RIGHT:
      if (mCurrentLane != mRightMostLane) {
        states.push_back(STATE_PREP_LANE_CHANGE_RIGHT);
        states.push_back(STATE_LANE_CHANGE_RIGHT);
      }
      break;

    default:
      break;
  }

  return states;
}

void Vehicle::generate_prediction(int prev_size) {
  mS += ((double)prev_size * TIME_INTERVAL * mVelocity);
}

vector<Vehicle> Vehicle::generate_trajectory(int state,
    map<int, Vehicle>& predictions) {
  vector<Vehicle> trajectory;

  switch (state) {
    case STATE_KEEP_LANE:
      trajectory = keep_lane_trajectory(predictions);
      break;

    case STATE_LANE_CHANGE_LEFT:
    case STATE_LANE_CHANGE_RIGHT:
      trajectory = lane_change_trajectory(state, predictions);
      break;

    case STATE_PREP_LANE_CHANGE_LEFT:
    case STATE_PREP_LANE_CHANGE_RIGHT:
      trajectory = prep_lane_change_trajectory(state, predictions);
      break;

    default:
      break;
  }

  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, Vehicle>& predictions) {
  vector<Vehicle> trajectory = {Vehicle(this->mCurrentLane, this->mS, this->mVelocity, this->mCurrentState)};

  vector<double> kinematics = get_kinematics(this->mCurrentLane, predictions);
  double new_s = kinematics[0];
  double new_v = kinematics[1];

  trajectory.emplace_back(this->mCurrentLane, new_s, new_v, STATE_KEEP_LANE);

  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(int state,
    map<int, Vehicle>& predictions) {
  vector<Vehicle> trajectory;
  int new_lane = this->mCurrentLane + lane_direction[state];
  Vehicle next_lane_vehicle;
  map<int, Vehicle>::const_iterator it;

  for (it = predictions.begin(); it != predictions.end(); ++it) {
    next_lane_vehicle = it->second;
    if (next_lane_vehicle.mS == this->mS &&
        next_lane_vehicle.mCurrentLane == new_lane) {
      // There's a potential collision in new_lane, return empty trajectory.
      return trajectory;
    }
  }

  trajectory.emplace_back(Vehicle(this->mCurrentLane, this->mS, this->mVelocity, this->mCurrentState));
  vector<double> kinematics = get_kinematics(new_lane, predictions);
  trajectory.emplace_back(Vehicle(new_lane, kinematics[0], kinematics[1], state));

  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(int state,
    map<int, Vehicle>& predictions) {
  double new_s;
  double new_v;
  vector<Vehicle> trajectory = {Vehicle(this->mCurrentLane, this->mS, this->mVelocity, this->mCurrentState)};

  vector<double> curr_lane_kinematics = get_kinematics(this->mCurrentLane, predictions);

  int new_lane = this->mCurrentLane + lane_direction[state];
  Vehicle vehicle_behind;
  bool vehicle_behind_too_close = get_vehicle_behind(new_lane, predictions, vehicle_behind);
  if (vehicle_behind_too_close) {
    new_s = curr_lane_kinematics[0];
    new_v = curr_lane_kinematics[1];
  } else {
    vector<double> new_lane_kinematics = get_kinematics(new_lane, predictions);
    if (new_lane_kinematics[1] < curr_lane_kinematics[1]) {
      new_s = curr_lane_kinematics[0];
      new_v = curr_lane_kinematics[1];
    } else {
      new_s = new_lane_kinematics[0];
      new_v = new_lane_kinematics[1];
    }
  }

  trajectory.emplace_back(this->mCurrentLane, new_s, new_v, state);

  return trajectory;
}

vector<double> Vehicle::get_kinematics(int lane,
    map<int, Vehicle>& predictions) {
  Vehicle vehicle_ahead;
  double new_position;
  double new_velocity;

  bool vehicle_ahead_too_close = get_vehicle_ahead(lane, predictions, vehicle_ahead);
  if (vehicle_ahead_too_close) {
    new_velocity = vehicle_ahead.mVelocity;
    /* Slow down our ego vehicle, but try to avoid jerk */
    if (fabs(new_velocity - this->mVelocity) > MAX_ALLOWED_ACCEL) {
      new_velocity = this->mVelocity - MAX_ALLOWED_ACCEL;
    }
  } else {
    new_velocity = this->mVelocity;

    /* Accelerate our ego vehicle, but try to avoid jerk */
    if (this->mVelocity < this->mMaxSpeedLimit) {
      new_velocity += MAX_ALLOWED_ACCEL;
    }
  }

  new_position = this->mS + TIME_INTERVAL * new_velocity;

  return {new_position, new_velocity};
}

bool Vehicle::get_vehicle_ahead(int lane,
    map<int, Vehicle>& predictions, Vehicle &ahead) {
  /*
   * Returns a true if a vehicle is found ahead of the current vehicle, false otherwise.
   * The reference "ahead" is updated if a vehicle is found.
   */
  bool found(false);
  Vehicle temp_vehicle;
  map<int, Vehicle>::const_iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;

    if (temp_vehicle.mD < (LANE_HALF_WIDTH + LANE_WIDTH * lane + LANE_HALF_WIDTH) &&
        temp_vehicle.mD > (LANE_HALF_WIDTH + LANE_WIDTH * lane - LANE_HALF_WIDTH)) {
      if (temp_vehicle.mS > this->mS &&
          (temp_vehicle.mS - this->mS) < SAFE_DISTANCE_AHEAD_IN_S) {
        ahead = temp_vehicle;
        found = true;
        break;
      }
    }
  }

  return found;
}

bool Vehicle::get_vehicle_behind(int lane,
    map<int, Vehicle> &predictions, Vehicle &behind) {
  /*
   * Returns a true if a vehicle is found behind of the current vehicle, false otherwise.
   * The reference "behind" is updated if a vehicle is found.
   */
  bool found(false);
  Vehicle temp_vehicle;
  map<int, Vehicle>::const_iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;

    if (temp_vehicle.mD < (LANE_HALF_WIDTH + LANE_WIDTH * lane + LANE_HALF_WIDTH) &&
        temp_vehicle.mD > (LANE_HALF_WIDTH + LANE_WIDTH * lane - LANE_HALF_WIDTH)) {
      if (this->mS > temp_vehicle.mS &&
          (this->mS - temp_vehicle.mS) < SAFE_DISTANCE_BEHIND_IN_S) {
        behind = temp_vehicle;
        found = true;
        break;
      }
    }
  }

  return found;
}