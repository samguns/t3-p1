//
// Created by Sam on 8/6/2018.
//

#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

#include <vector>
#include <map>
#include "vehicle.h"

using namespace std;

double calculate_cost(const vector<Vehicle>& trajectory,
                      map<int, Vehicle>& predictions);

#endif //PATH_PLANNING_COST_H
