#ifndef _PREDICTION_H_
#define _PREDICTION_H_

#include <map>
#include <vector>
#include "road.h"
#include "vehicle.h"

/**
 * Prediction module is for predicting non-ego cars' trajectories.
 */
class Prediction
{
public:
    Prediction(const Road& road, double pred_horizon_sec, double pred_resolution_sec);

    /**
     * @param sensor_fusion sensor's data received from simulator
     * @return a map of vehicle ID => its predicted trajectory.
     */
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

    /**
     * Parses raw sensor fusion data into an internal structure.
     */
    SF parse_sf(const vector<double> sf);

    /**
     * Predicts trajectory for the given vehicle v.
     */
    vector<Vehicle> predict_trajectory(const Vehicle& v);
};

#endif // _PREDICTION_H_