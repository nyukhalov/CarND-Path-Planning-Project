#ifndef _PREDICTION_H_
#define _PREDICTION_H_

#include <map>
#include <vector>
#include "road.h"
#include "vehicle.h"

class Prediction
{
public:
    Prediction(const Road& road, double pred_horizon_sec, double pred_resolution_sec);

    map<int, vector<Vehicle>> predict(const vector<vector<double>>& sensor_fusion);

private:
    struct SF {
        int id;
        double x;
        double y;
        double vx;
        double vy;
        double s;
        double d;
    };

    const Road& road;
    double pred_horizon_sec;
    double pred_resolution_sec;

    SF parse_sf(const vector<double> sf);

    vector<Vehicle> predict_vehicle(const Vehicle& v);
};

#endif // _PREDICTION_H_