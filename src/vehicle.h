//
// Created by Sam on 2018/8/4.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <vector>
#include <map>

#define SENSOR_FUSION_ID_IDX      0
#define SENSOR_FUSION_X_IDX       1
#define SENSOR_FUSION_Y_IDX       2
#define SENSOR_FUSION_VX_IDX      3
#define SENSOR_FUSION_VY_IDX      4
#define SENSOR_FUSION_S_IDX       5
#define SENSOR_FUSION_D_IDX       6

#define TIME_INTERVAL             0.02

#define SAFE_DISTANCE_IN_S        30
#define MAX_ALLOWED_ACCEL         0.224


using namespace std;

class Vehicle {
 public:
  Vehicle();
  Vehicle(vector<double> sensored_state);
  Vehicle(int lane, double s, double v, int state=STATE_INIT);
  virtual ~Vehicle() {};

  /**
   * @param max_speed   Maximum legal speed
   * @param lanes       Lane numbers in Frenet Coordinate, ordered from left to right
   */
  void configure(double max_speed, int currentLane, vector<int> lanes);

  /**
   * @param sensor_fusion   Input data of other vehicles from Sensor Fusion
   * @param goal_lane       Output target lane
   * @param goal_v          Output target velocity
   *
   * @note We predict other vehicles run on a constant velocity in 0.02s ahead of time,
   *       without abrupt lane changes.
   */
  void getNextBehavior(int prev_size, vector<vector<double>> sensor_fusion,
                       int& goal_lane, double& goal_v);

 private:
  int mCurrentState;
  int mCurrentLane;
  int mNumberOfLanes;
  int mLeftMostLane;
  int mRightMostLane;
  double mMaxSpeedLimit;

  double mS;
  double mD;
  double mVelocity;
  double mAcceleration;

  vector<int> successor_states();
  void generate_prediction(int prev_size);
  vector<Vehicle> generate_trajectory(int state, map<int, Vehicle>& predictions);
  vector<Vehicle> keep_lane_trajectory(map<int, Vehicle>& predictions);
  vector<Vehicle> lane_change_trajectory(int state, map<int, Vehicle>& predictions);
  vector<Vehicle> prep_lane_change_trajectory(int state, map<int, Vehicle>& predictions);
  vector<double> get_kinematics(int lane, map<int, Vehicle>& predictions);
  bool get_vehicle_ahead(int lane, map<int, Vehicle>& predictions, Vehicle& ahead);
  bool get_vehicle_behind(int lane, map<int, Vehicle>& predictions, Vehicle& behind);

  enum {
    STATE_INIT = 0,
    STATE_KEEP_LANE,
    STATE_PREP_LANE_CHANGE_LEFT,
    STATE_PREP_LANE_CHANGE_RIGHT,
    STATE_LANE_CHANGE_LEFT,
    STATE_LANE_CHANGE_RIGHT,
  };
};

#endif //PATH_PLANNING_VEHICLE_H
