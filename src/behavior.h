#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include <map>
#include <vector>
#include "vehicle.h"

using namespace std;

struct Target {
    int lane;
    double speed;
};

class Behavior {
public:
    Target plan(const Vehicle& self, map<int, vector<Vehicle>> predictions);
};

#endif //_BEHAVIOR_H_