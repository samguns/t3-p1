#include <utility>

//
// Created by Sam on 2018/8/4.
//

#include <math.h>
#include "vehicle.h"


Vehicle::Vehicle() :
  mCurrentState(STATE_INIT)
{}

Vehicle::Vehicle(vector<double> sensored_state) :
  mS(sensored_state[SENSOR_FUSION_S_IDX]),
  mD(sensored_state[SENSOR_FUSION_D_IDX]) {
  double vx = sensored_state[SENSOR_FUSION_VX_IDX];
  double vy = sensored_state[SENSOR_FUSION_VY_IDX];

  mVelocity = sqrt(vx*vx + vy*vy);
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
  vector<int> states = successor_states();
  for (const int& state : states) {
    vector<Vehicle> trajectory = generate_trajectory(state, predictions);
  }
}


vector<int> Vehicle::successor_states() {
  vector<int> states;

  states.push_back(STATE_KEEP_LANE);

  switch (mCurrentState) {
    case STATE_KEEP_LANE:
      states.push_back(STATE_PREP_LANE_CHANGE_LEFT);
      states.push_back(STATE_PREP_LANE_CHANGE_RIGHT);
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
    map<int, Vehicle>& predictions) {}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(int state,
    map<int, Vehicle>& predictions) {}

vector<double> Vehicle::get_kinematics(int lane,
    map<int, Vehicle>& predictions) {
  Vehicle vehicle_ahead;
  // Vehicle vehicle_behind;
  double new_position;
  double new_velocity;

  bool vehicle_ahead_too_close = get_vehicle_ahead(lane, predictions, vehicle_ahead);
  //bool vehicle_behind_too_close = get_vehicle_behind(lane, predictions, vehicle_behind);
  if (vehicle_ahead_too_close) {
    new_velocity = vehicle_ahead.mVelocity;
    /* Slow down our ego vehicle, but try to avoid jerk */
    if (fabs(new_velocity - this->mVelocity) > MAX_ALLOWED_ACCEL) {
      new_velocity = this->mVelocity - MAX_ALLOWED_ACCEL;
    }
  } else {
    new_velocity = this->mVelocity;

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

    if (temp_vehicle.mD < (2 + 4 * lane + 2) && temp_vehicle.mD > (2 + 4 * lane - 2)) {
      if (temp_vehicle.mS > this->mS &&
          (temp_vehicle.mS - this->mS) < SAFE_DISTANCE_IN_S) {
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

    if (temp_vehicle.mD < (2 + 4 * lane + 2) && temp_vehicle.mD > (2 + 4 * lane - 2)) {
      if (this->mS > temp_vehicle.mS &&
          (this->mS - temp_vehicle.mS) < SAFE_DISTANCE_IN_S) {
        behind = temp_vehicle;
        found = true;
        break;
      }
    }
  }

  return found;
}