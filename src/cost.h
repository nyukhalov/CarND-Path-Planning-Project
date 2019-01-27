#ifndef _COST_H_
#define _COST_H_

#include <iostream>
#include <vector>
#include <map>
#include "vehicle.h"
#include "utils.h"

using namespace std;

namespace
{
namespace cost
{

double collision_cost(const vector<Vehicle>& trajectory, const map<int, vector<Vehicle>>& predictions)
{
    double min_dist = 5;
    int n_iter = trajectory.size();
    for(int i=0; i<n_iter; i++) {
        auto self = trajectory.at(i);
        for(auto it=predictions.begin(); it != predictions.end(); ++it) {
            auto v = it->second.at(i);
            double dist = utils::distance(self.x, self.y, v.x, v.y);
            if (dist < min_dist) return 1;
        }
    }
    // no collisions
    return 0;
}

} // namespace cost    
} // namespace

#endif // _COST_H_