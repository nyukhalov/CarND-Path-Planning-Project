#include "behavior.h"
#include "road.h"
#include "utils.h"
#include "cost.h"

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
    // if (true) return naive_plan(self, predictions);

    std::cout << "Run behavior planner for state=" << state << std::endl;
    auto sstates = fsm::successor_states(state);
    double min_cost = 99999999;
    vector<Vehicle> best_trajectory;
    string best_state = fsm::STATE_KL;

    for (auto it = sstates.begin(); it != sstates.end(); ++it)
    {
        string s = *it;
        vector<Vehicle> trajectory = generate_trajectory(self, s, predictions);
        if (!trajectory.empty())
        {
            double cost = calculate_cost(self, trajectory, predictions);
            std::cout << "plan: ======== " << s << ": cost=" << cost << std::endl;
            if (cost < min_cost)
            {
                min_cost = cost;
                best_trajectory = trajectory;
                best_state = s;
            }
        }
    }

    this->state = best_state;
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
    // std::cout << "keep_lane_trajectory: self_lane=" << self_lane << std::endl;
    return rough_trajectory(self, predictions, self_lane);
}

vector<Vehicle> Behavior::change_lane_left_trajectory(const Vehicle &self, const map<int, vector<Vehicle>> predictions)
{
    int self_lane = road.get_lane(self.d);

    // cannot change lane as the vehicle is on the most left lane.
    if (!road.can_change_lane_left(self_lane)) {
        std::cout << "change_lane_left_trajectory: cannot change lane: self_lane=" << self_lane << std::endl;
        return {};
    }

    return rough_trajectory(self, predictions, self_lane - 1);
}

vector<Vehicle> Behavior::change_lane_right_trajectory(const Vehicle &self, const map<int, vector<Vehicle>> predictions)
{
    int self_lane = road.get_lane(self.d);
    assert(self_lane != -1);

    // cannot change lane as the vehicle is on the most right lane.
    if (!road.can_change_lane_right(self_lane)) {
        std::cout << "change_lane_right_trajectory: cannot change lane: self_lane=" << self_lane << std::endl;
        return {};
    }

    return rough_trajectory(self, predictions, self_lane + 1);
}

vector<Vehicle> Behavior::rough_trajectory(const Vehicle& self, const map<int, vector<Vehicle>> predictions, int target_lane) 
{
    vector<Vehicle> trajectory;

    double max_angle = 60; // degrees
    double max_accel = 10; // m/s^2

    double target_d = road.lane_center(target_lane);
    // std::cout << "rough_trajectory: target_d=" << target_d << std::endl;

    double car_d = self.d;
    double car_s = self.s;
    double car_v = MPH2mps(self.speed); // convert to velocity in meters per second

    double dt = pred_resolution_sec; // time between iterations

    int num_iter = pred_horizon_sec / pred_resolution_sec;
    for(int i=0; i<num_iter; i++)
    {
        // std::cout << "rough_trajectory: iter=" << i+1 << " car={s=" << car_s << ", d=" << car_d << ", v=" << car_v << "}" << std::endl;

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
        double dd = target_d - car_d;
        if (dd > max_dd)
        {
            dd = max_dd;
        } else if (dd < -max_dd) {
            dd = -max_dd;
        }
        double ds = sqrt(dist*dist - dd*dd);

        // std::cout << "dist=" << dist << ", max_dd=" << max_dd << ", dd=" << dd << ", ds=" << ds << std::endl;

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
    double w_collision = 1;

    vector<string> cf_name_list
    = {"collision_cost"};

    vector< function<double (const vector<Vehicle> &, const map<int, vector<Vehicle>> &)>> cf_list 
    = {cost::collision_cost};

    vector<double> weight_list = {w_collision};

    double cost = 0;
    for(int i=0; i<weight_list.size(); i++) {
        string name = cf_name_list[i];
        double w = weight_list[i];
        auto cf = cf_list[i];
        double c = cf(trajectory, predictions);
        double weighted_cost = w * c;  
        cost += weighted_cost;
        std::cout << name << ": W=" << w << ", cost=" << c << ", weighed_cost=" << weighted_cost << std::endl;
    } 

    return cost;
}

Target Behavior::build_target(const vector<Vehicle> &trajectory)
{
    Vehicle last_state = trajectory[trajectory.size() - 1];

    Target t;
    t.lane = road.get_lane(last_state.d);
    t.speed = last_state.speed;

    std::cout << "Build target for vehicle(d=" << last_state.d << "): t.lane=" << t.lane << ", t.speed=" << t.speed << std::endl;

    assert(t.lane != -1);

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