#ifndef _TRAJ_GEN_H
#define _TRAJ_GEN_H

#include "road.h"
#include "vehicle.h"
#include "behavior.h"

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const Road& road, double update_freq_sec, double max_accel, double speed_limit);

    vector<Vehicle> generate(const vector<Vehicle>& prev_trajectory, const Vehicle& self, const map<int, vector<Vehicle>>& predictions, Target target);

private:
    const Road& road;
    double update_freq_sec;
    double max_accel;
    double velocity_limit;

    double cur_velocity;
    double dist_fc;
};

#endif // _TRAJ_GEN_H