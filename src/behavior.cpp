#include "behavior.h"
#include "road.h"

Target Behavior::plan(const Vehicle &self, map<int, vector<Vehicle>> predictions)
{
    Target t;
    t.lane = 1;
    t.speed = 50;

    double min_dist = 99999;
    
    for(map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        Vehicle v = it->second[0];
        if (road::is_within_lane(t.lane, v.d))
        {
            if (v.s > self.s)
            {
                double dist = v.s - self.s;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }
    }

    if (min_dist < 20) {
        // t.speed = 30;
        t.lane = 0;
    }

    return t;
};