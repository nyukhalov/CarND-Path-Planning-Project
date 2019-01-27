#include "behavior.h"
#include "road.h"
#include "utils.h"

#include <iostream>

using namespace utils;

Behavior::Behavior(double pred_horizon_sec, double pred_resolution_sec, double speed_limit, const Road& road)
: road(road) {
    this->pred_horizon_sec = pred_horizon_sec;
    this->pred_resolution_sec = pred_resolution_sec;
    this->velocity_limit = MPH2mps(speed_limit);
    this->preferred_buffer = 5.0; // 5 meters
    this->state = fsm::STATE_KL;
}

Target Behavior::plan(const Vehicle &self, map<int, vector<Vehicle>> predictions)
{
    if (true) return naive_plan(self, predictions);

    std::cout << "Run behavior planner for state=" << state << std::endl;
    auto sstates = fsm::successor_states(state);
    double min_cost = 99999999;
    vector<Vehicle> best_trajectory;
    string best_state = fsm::STATE_KL;

    for (auto it = sstates.begin(); it != sstates.end(); ++it)
    {
        string state = *it;
        vector<Vehicle> trajectory = generate_trajectory(self, state, predictions);
        if (!trajectory.empty())
        {
            double cost = calculate_cost(self, trajectory, predictions);
            if (cost < min_cost)
            {
                min_cost = cost;
                best_trajectory = trajectory;
                best_state = state;
            }
        }
    }

    std::cout << "The best next state=" << best_state << std::endl;
    return build_target(best_trajectory);
};

vector<Vehicle> Behavior::generate_trajectory(const Vehicle &self, const string &state, const map<int, vector<Vehicle>> predictions)
{
    vector<Vehicle> trajectory;

    if (fsm::is_KL(state))
    {
        trajectory = keep_lane_trajectory(self, predictions);
    }
    else if (fsm::is_CLL(state))
    {
        trajectory = change_lane_left_trajectory(self, predictions);
    }
    else if (fsm::is_CLR(state))
    {
        trajectory = change_lane_right_trajectory(self, predictions);
    }

    return trajectory;
}

vector<Vehicle> Behavior::keep_lane_trajectory(const Vehicle &self, const map<int, vector<Vehicle>> predictions)
{
    int self_lane = road.get_lane(self.d);
    return rough_trajectory(self, predictions, self_lane);
}

vector<Vehicle> Behavior::change_lane_left_trajectory(const Vehicle &self, const map<int, vector<Vehicle>> predictions)
{
    int self_lane = road.get_lane(self.d);

    // cannot change lane as the vehicle is on the most left lane.
    if (self_lane <= 0)
        return {};

    return rough_trajectory(self, predictions, self_lane - 1);
}

vector<Vehicle> Behavior::change_lane_right_trajectory(const Vehicle &self, const map<int, vector<Vehicle>> predictions)
{
    int self_lane = road.get_lane(self.d);

    // cannot change lane as the vehicle is on the most right lane.
    if (self_lane >= num_lanes - 1)
        return {};

    return rough_trajectory(self, predictions, self_lane + 1);
}

vector<Vehicle> Behavior::rough_trajectory(const Vehicle& self, const map<int, vector<Vehicle>> predictions, int target_lane) 
{
    vector<Vehicle> trajectory;

    double max_angle = 60; // degrees
    double max_accel = 10; // m/s^2

    double target_d = road.lane_center(target_lane);

    double car_d = self.d;
    double car_s = self.s;
    double car_v = MPH2mps(self.speed); // convert to velocity in meters per second

    double dt = pred_resolution_sec; // time between iteration

    int num_iter = pred_horizon_sec / pred_resolution_sec;
    while (num_iter-- > 0)
    {
        // finding max safe velocity
        double car_v_max = car_v + dt*max_accel;

        Vehicle vehicle_ahead;
        Vehicle vehicle_behind;

        // if (get_vehicle_ahead(predictions, self.get_lane(), vehicle_ahead)) {
        // double ds = vehicle_ahead.s - self.s;
        // double max_velocity_in_front = (ds - preferred_buffer) + MPH2mps(vehicle_ahead.speed) - 0.5 * max_accel;
        // car_v = min(min(max_velocity_in_front, car_v_max), velocity_limit);
        // } else {
        car_v = min(car_v_max, velocity_limit);
        // }

        // calculation
        double dist = car_v * dt;
        double max_dd = dist * cos(deg2rad(max_angle));
        double dd = fabs(target_d - car_d);
        if (dd > max_dd)
        {
            dd = max_dd;
        }
        double ds = sqrt(dist * dist - dd * dd);

        car_d += dd;
        car_s += ds;

        double car_yaw = self.yaw; // TODO: calculate car's yaw

        auto xy = road.get_xy(car_s, car_d);
        double speed = utils::mps2MPH(car_v);
        auto v = Vehicle(xy[0], xy[1], car_s, car_d, car_yaw, speed);
        trajectory.push_back(v);
    }

    return trajectory;    
}

double Behavior::calculate_cost(const Vehicle &self, const vector<Vehicle> &trajectory, const map<int, vector<Vehicle>> predictions)
{
    return 0;
}

Target Behavior::build_target(const vector<Vehicle> &trajectory)
{
    Vehicle last_state = trajectory[trajectory.size() - 1];

    Target t;
    t.lane = road.get_lane(last_state.d);
    t.speed = last_state.speed;

    return t;
}

Target Behavior::naive_plan(const Vehicle &self, map<int, vector<Vehicle>> predictions)
{
    Target t;
    t.lane = 1;
    t.speed = 50;

    double min_dist = 99999;

    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it)
    {
        Vehicle v = it->second[0];
        if (road.is_within_lane(t.lane, v.d))
        {
            if (v.s > self.s)
            {
                double dist = v.s - self.s;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }
    }

    if (min_dist < 20)
    {
        // t.speed = 30;
        t.lane = 0;
    }

    return t;
}