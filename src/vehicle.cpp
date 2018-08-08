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
  mCurrentState(STATE_INIT), mMinStayInLaneCount(0), mAcceleration(0)
{}

Vehicle::Vehicle(vector<double> sensored_state) :
  mS(sensored_state[SENSOR_FUSION_S_IDX]),
  mD(sensored_state[SENSOR_FUSION_D_IDX]) {
  double vx = sensored_state[SENSOR_FUSION_VX_IDX];
  double vy = sensored_state[SENSOR_FUSION_VY_IDX];

  mVelocity = sqrt(vx*vx + vy*vy);

  cout << "ID: " << sensored_state[SENSOR_FUSION_ID_IDX] << " mD: " << mD << endl;

  for (int lane = 0; lane < MAX_NOF_LANES; lane++) {
    if (mD < (LANE_HALF_WIDTH + LANE_WIDTH * lane + LANE_HALF_WIDTH) &&
        mD > (LANE_HALF_WIDTH + LANE_WIDTH * lane - LANE_HALF_WIDTH)) {
      mCurrentLane = lane;
      cout << "mCurrentLane: " << mCurrentLane << " lane: " << lane << endl;
    }
  }
  cout << endl;
}

Vehicle::Vehicle(int lane, double s, double v, double a, int state) :
  mCurrentLane(lane), mS(s), mVelocity(v), mCurrentState(state),
  mAcceleration(a) {}

void Vehicle::configure(double max_speed, int currentLane, vector<int> lanes) {
  mMaxSpeedLimit = max_speed;
  mNumberOfLanes = lanes.size();
  mLeftMostLane = lanes[0];
  mRightMostLane = lanes[mNumberOfLanes-1];
  mCurrentLane = currentLane;
}

void Vehicle::update(double s, double v) {
  mS = s;
  //mVelocity = v;
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

  /* Future endpoint is the 1 second ahead from where our
   * previous path ends.
   */

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
  this->mAcceleration = best_trajectory[1].getAcceleration();

  /* Start a timer to protect ego vehicle from seemingly consecutive lane changing.
   * That is, at least MIN_STAY_IN_LANE_PERIOD (4) seconds later can we attemp to
   * change lane.
   */
  if (this->mCurrentState == STATE_LANE_CHANGE_LEFT ||
      this->mCurrentState == STATE_LANE_CHANGE_RIGHT) {
    this->mMinStayInLaneCount = prev_size + MIN_STAY_IN_LANE_PERIOD * NOF_PATH_POINTS;
  }

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

double Vehicle::getAcceleration() {
  return mAcceleration;
}


vector<int> Vehicle::successor_states() {
  vector<int> states;

  states.push_back(STATE_KEEP_LANE);

  if (mMinStayInLaneCount > 0) {
    mMinStayInLaneCount--;
    return states;
  }

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
  vector<Vehicle> trajectory = {Vehicle(this->mCurrentLane, this->mS,
                                this->mVelocity, this->mAcceleration, this->mCurrentState)};

  vector<double> kinematics = get_kinematics(this->mCurrentLane, predictions);
  double new_s = kinematics[0];
  double new_v = kinematics[1];
  double new_a = kinematics[2];

  trajectory.emplace_back(this->mCurrentLane, new_s, new_v, new_a, STATE_KEEP_LANE);

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
    if (next_lane_vehicle.mCurrentLane == new_lane) {
      if (fabs(this->mS - next_lane_vehicle.mS) <= SAFE_DISTANCE_IN_S) {
        // There's a potential collision in new_lane, return empty trajectory.
        return trajectory;
      }
    }
  }

  trajectory.emplace_back(Vehicle(this->mCurrentLane, this->mS,
                          this->mVelocity, this->mAcceleration, this->mCurrentState));
  vector<double> kinematics = get_kinematics(new_lane, predictions);
  trajectory.emplace_back(Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));

  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(int state,
    map<int, Vehicle>& predictions) {
  double new_s;
  double new_v;
  double new_a;
  vector<Vehicle> trajectory = {Vehicle(this->mCurrentLane, this->mS,
                                this->mVelocity, this->mAcceleration, this->mCurrentState)};

  vector<double> curr_lane_kinematics = get_kinematics(this->mCurrentLane, predictions);

  int new_lane = this->mCurrentLane + lane_direction[state];
  Vehicle vehicle_behind;
  bool vehicle_behind_too_close = get_vehicle_behind(this->mCurrentLane, predictions, vehicle_behind);
  if (vehicle_behind_too_close) {
    new_s = curr_lane_kinematics[0];
    new_v = curr_lane_kinematics[1];
    new_a = curr_lane_kinematics[2];
  } else {
    vector<double> new_lane_kinematics = get_kinematics(new_lane, predictions);
    if (new_lane_kinematics[1] < curr_lane_kinematics[1]) {
      new_s = curr_lane_kinematics[0];
      new_v = curr_lane_kinematics[1];
      new_a = curr_lane_kinematics[2];
    } else {
      new_s = new_lane_kinematics[0];
      new_v = new_lane_kinematics[1];
      new_a = new_lane_kinematics[2];
    }
  }

  trajectory.emplace_back(this->mCurrentLane, new_s, new_v, new_a, state);

  return trajectory;
}

vector<double> Vehicle::get_kinematics(int lane,
    map<int, Vehicle>& predictions) {
  Vehicle vehicle_ahead;
  double new_position;
  double new_velocity;
  double new_accel;

  double max_velocity_accel_limit = this->mVelocity + MAX_ALLOWED_ACCEL;

  bool vehicle_ahead_too_close = get_vehicle_ahead(lane, predictions, vehicle_ahead);
  if (vehicle_ahead_too_close) {
    double max_velocity_in_front = (vehicle_ahead.mS - this->mS) +
                          vehicle_ahead.mVelocity - this->mAcceleration / 2;
    new_velocity = min(min(max_velocity_accel_limit, max_velocity_in_front), this->mMaxSpeedLimit);

  } else {
    new_velocity = min(max_velocity_accel_limit, this->mMaxSpeedLimit);
  }

  new_accel = (new_velocity - this->mVelocity) / (NOF_PATH_POINTS * TIME_INTERVAL);

  /* distance = v*t + a*t*t/2, if acceleration is constant.
   * Here we have t = 1.
   */
  new_position = this->mS + new_velocity + new_accel / 2;

  return {new_position, new_velocity, new_accel};
}

bool Vehicle::get_vehicle_ahead(int lane,
    map<int, Vehicle>& predictions, Vehicle &ahead) {
  /*
   * Returns a true if a vehicle is found ahead of us, false otherwise.
   * The reference "ahead" is updated to the closest vehicle.
   */
  double min_distance = SAFE_DISTANCE_IN_S;
  bool found(false);
  Vehicle temp_vehicle;
  map<int, Vehicle>::const_iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;

    if (temp_vehicle.mD < (LANE_HALF_WIDTH + LANE_WIDTH * lane + LANE_HALF_WIDTH) &&
        temp_vehicle.mD > (LANE_HALF_WIDTH + LANE_WIDTH * lane - LANE_HALF_WIDTH)) {
      if (temp_vehicle.mCurrentLane != lane) {
        cout << "temp_vehicle.mD: " << temp_vehicle.mD << " currentLane " << temp_vehicle.mCurrentLane << " lane: "
             << lane << endl;
      }
      if (temp_vehicle.mS > this->mS) {
        double distance = temp_vehicle.mS - this->mS;
        if (distance < min_distance) {
          ahead = temp_vehicle;
          min_distance = distance;
          found = true;
        }
      }
    }
  }

  return found;
}

bool Vehicle::get_vehicle_behind(int lane,
    map<int, Vehicle> &predictions, Vehicle &behind) {
  /*
   * Returns a true if a vehicle is found behind us, false otherwise.
   * The reference "behind" is updated to the closest vehicle.
   */
  double min_distance = SAFE_DISTANCE_IN_S;
  bool found(false);
  Vehicle temp_vehicle;
  map<int, Vehicle>::const_iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second;

    if (temp_vehicle.mD < (LANE_HALF_WIDTH + LANE_WIDTH * lane + LANE_HALF_WIDTH) &&
        temp_vehicle.mD > (LANE_HALF_WIDTH + LANE_WIDTH * lane - LANE_HALF_WIDTH)) {
      if (temp_vehicle.mCurrentLane != lane) {
        cout << "temp_vehicle.mD: " << temp_vehicle.mD << " currentLane " << temp_vehicle.mCurrentLane << " lane: "
             << lane << endl;
      }
      if (this->mS > temp_vehicle.mS) {
        double distance =  this->mS - temp_vehicle.mS;
        if (distance < min_distance) {
          behind = temp_vehicle;
          min_distance = distance;
          found = true;
        }
      }
    }
  }

  return found;
}