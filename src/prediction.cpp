#include "prediction.h"
#include <math.h>
#include <iostream>
#include "utils.h"

Prediction::Prediction(const Road &road, double pred_horizon_sec, double pred_resolution_sec)
    : road(road)
{
    this->pred_horizon_sec = pred_horizon_sec;
    this->pred_resolution_sec = pred_resolution_sec;
}

map<int, vector<Vehicle>> Prediction::predict(const vector<vector<double>> &sensor_fusion)
{
    map<int, vector<Vehicle>> predictions;
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        auto sf = parse_sf(sensor_fusion[i]);
        double vel = sqrt(sf.vx*sf.vx + sf.vy*sf.vy);
        double speed = utils::mps2MPH(vel);
        auto v = Vehicle(sf.x, sf.y, sf.s, sf.d, 0, speed);
        predictions[sf.id] = predict_vehicle(v);
    }
    return predictions;
}

Prediction::SF Prediction::parse_sf(const vector<double> raw_sf)
{
    Prediction::SF sf;
    // id, x, y, vx, vy, s, d
    sf.id = (int)raw_sf[0];
    sf.x = raw_sf[1];
    sf.y = raw_sf[2];
    sf.vx = raw_sf[3];
    sf.vy = raw_sf[4];
    sf.s = raw_sf[5];
    sf.d = raw_sf[6];
    return sf;
}

vector<Vehicle> Prediction::predict_vehicle(const Vehicle& v)
{
    /**
     * Simple prediction: moving along the current lane with a constant speed
     */
    int n_iter = pred_horizon_sec / pred_resolution_sec;
    assert(n_iter > 0);

    vector<Vehicle> trajectory;

    double dt = pred_resolution_sec; // time between iterations
    int cur_lane = road.get_lane(v.d);
    double car_yaw = v.yaw;
    double car_d = road.lane_center(cur_lane);
    double car_s = v.s;
    double ds = utils::MPH2mps(v.speed) * dt;

    trajectory.push_back(v);
    for(int i=0; i<n_iter-1; i++) {
        car_s += ds;
        auto xy = road.get_xy(car_s, car_d);
        auto pred = Vehicle(xy[0], xy[1], car_s, car_d, car_yaw, v.speed);
        trajectory.push_back(pred);
    }

    return trajectory;
}