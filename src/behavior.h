#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include <map>
#include <vector>
#include "vehicle.h"
#include "fsm.h"
#include "road.h"

using namespace std;

struct Target {
    int lane;
    double speed;
};

class Behavior {
public:

    // speed_limit - max allowed vehicle's speed in miles per hour
    Behavior(double pred_horizon_sec, double pred_resolution_sec, double goal_at_sec, double speed_limit, double max_accel, const Road& road);

    // can plan up to 10 seconds ahead
    Target plan(const Vehicle& self, map<int, vector<Vehicle>> predictions);

private:
    double pred_horizon_sec;
    double pred_resolution_sec;
    double goal_at_sec;
    double velocity_limit; // max allowed vehicle's velocity in meters per second
    double max_accel;
    double preferred_buffer;
    const Road& road;

    string state;

    vector<Vehicle> generate_trajectory(const Vehicle& self, const string& state, const map<int, vector<Vehicle>> predictions);

    vector<Vehicle> keep_lane_trajectory(const Vehicle& self, const map<int, vector<Vehicle>> predictions);

    vector<Vehicle> change_lane_left_trajectory(const Vehicle& self, const map<int, vector<Vehicle>> predictions);

    vector<Vehicle> change_lane_right_trajectory(const Vehicle& self, const map<int, vector<Vehicle>> predictions);

    vector<Vehicle> rough_trajectory(const Vehicle& self, const map<int, vector<Vehicle>> predictions, int target_lane);

    bool get_vehicle_ahead(int iter, double car_s, double car_d, const map<int, vector<Vehicle>> predictions, int* id, Vehicle& vehicle_ahead);

    double calculate_cost(const Vehicle& self, const Target& target, const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>> predictions);

    Target build_target(const vector<Vehicle>& trajectory);

    Target naive_plan(const Vehicle& self, map<int, vector<Vehicle>> predictions);
};

#endif //_BEHAVIOR_H_