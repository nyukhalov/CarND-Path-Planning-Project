#ifndef _TRAJ_GEN_H
#define _TRAJ_GEN_H

#include "road.h"
#include "vehicle.h"
#include "behavior.h"

// Trajectory generator module.
class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const Road& road, double update_freq_sec, double max_accel, double speed_limit);

    // generates a trajectory for the given localization data (self), predictions and target.
    vector<Vehicle> generate(
        const vector<Vehicle>& prev_trajectory, 
        const Vehicle& self, 
        const map<int, vector<Vehicle>>& predictions, 
        Target target
    );

private:
    const Road& road;
    double update_freq_sec;
    double max_accel;
    double velocity_limit; // velocity in meters per second

    double cur_velocity; // desired ego-car's velocity at i-th iteration
    double dist_fc; // last distance to the front-center vehicle

    vector<double> get_last_position(const vector<Vehicle> &prev_trajectory, const Vehicle &self);
};

#endif // _TRAJ_GEN_H