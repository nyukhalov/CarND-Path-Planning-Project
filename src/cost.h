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
    int n_iter = trajectory.size();
    for(int i=0; i<n_iter; i++) {
        auto self = trajectory.at(i);
        for(auto it=predictions.begin(); it != predictions.end(); ++it) {
            auto v = it->second.at(i);

            double self_l = self.d - 0.5*Vehicle::WIDTH;
            double self_r = self.d + 0.5*Vehicle::WIDTH;
            double self_t = self.s + 0.5*Vehicle::LENGTH;
            double self_b = self.s - 0.5*Vehicle::LENGTH;

            double v_l = v.d - 0.5*Vehicle::WIDTH;
            double v_r = v.d + 0.5*Vehicle::WIDTH;
            double v_t = v.s + 0.5*Vehicle::LENGTH;
            double v_b = v.s - 0.5*Vehicle::LENGTH;

            // https://stackoverflow.com/questions/306316/determine-if-two-rectangles-overlap-each-other
            bool is_collision = self_l < v_r && self_r > v_l && self_t > v_b && self_b < v_t;
            if (is_collision) {
                // std::cout << "collision_cost: detected collision" << std::endl;
                return 1;
            }
        }
    }
    // no collisions
    return 0;
}

} // namespace cost    
} // namespace

#endif // _COST_H_