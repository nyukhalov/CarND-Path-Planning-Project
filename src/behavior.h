#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include <map>
#include <vector>
#include "vehicle.h"
#include "fsm.h"
#include "road.h"

using namespace std;

// Target is the output of the behavior module.
struct Target {
    int lane; // a lane number the ego-car must move to
    double speed; // speed the ego-car must move at (if > 0)
    int vehicle_id; // the ID of a non-ego vehicle to follow (use only if it's >= 0)
};

// Implementation of the behavior module
class Behavior {
public:

    // speed_limit - max allowed vehicle's speed in miles per hour
    Behavior(double pred_horizon_sec, double pred_resolution_sec, double speed_limit, double max_accel, const Road& road);

    // for the give ego-car self and predictions determines the next highlevel target.
    Target plan(const Vehicle& self, const map<int, vector<Vehicle>>& predictions);

private:
    double pred_horizon_sec;
    double pred_resolution_sec;
    double velocity_limit; // max allowed vehicle's velocity in meters per second
    double max_accel;
    double preferred_buffer;
    const Road& road;

    string state; // current FMS state

    // generates a trajectory for the given ego car and FSM state
    vector<Vehicle> generate_trajectory(const Vehicle& self, const string& state, const map<int, vector<Vehicle>>& predictions);

    vector<Vehicle> keep_lane_trajectory(const Vehicle& self, const map<int, vector<Vehicle>>& predictions);

    vector<Vehicle> change_lane_left_trajectory(const Vehicle& self, const map<int, vector<Vehicle>>& predictions);

    vector<Vehicle> change_lane_right_trajectory(const Vehicle& self, const map<int, vector<Vehicle>>& predictions);

    // generates a rough trajectory for the given ego-car. the car must end up on the target_lane lane in the end of the trajectory
    vector<Vehicle> rough_trajectory(const Vehicle& self, const map<int, vector<Vehicle>>& predictions, int target_lane);

    // find the front center vehicle for given car's s and d coordinates at interation iter.
    bool get_vehicle_ahead(int iter, double car_s, double car_d, const map<int, vector<Vehicle>>& predictions, int* id, Vehicle& vehicle_ahead);

    // calculate cost for the given trajectory.
    double calculate_cost(const Vehicle& self, const Target& target, const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>>& predictions);

    // builds a target object from the given trajectory.
    Target build_target(const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>>& predictions);
};

#endif //_BEHAVIOR_H_