#include "behavior.h"
#include "road.h"
#include "utils.h"
#include "cost.h"

#include <iostream>
#include <sstream>

using namespace utils;

Behavior::Behavior(double pred_horizon_sec, double pred_resolution_sec, double speed_limit, double max_accel, const Road& road)
: road(road) {
    this->pred_horizon_sec = pred_horizon_sec;
    this->pred_resolution_sec = pred_resolution_sec;
    this->velocity_limit = MPH2mps(speed_limit);
    this->max_accel = max_accel;
    this->preferred_buffer = 4; // meters
    this->state = fsm::STATE_KL;
}

Target Behavior::plan(const Vehicle &self, const map<int, vector<Vehicle>>& predictions)
{
    std::cout << "Run behavior planner for state=" << state << std::endl;
    auto sstates = fsm::successor_states(state);
    double min_cost = 99999999;
    string best_state = fsm::STATE_KL;
    Target best_target;

/*
    // print prediction trajectories (for debug purposes)
    for(auto it=predictions.begin(); it != predictions.end(); ++it)
    {
        int id = it->first;
        auto trajectory = it->second;
        std::ostringstream stringStream;
        stringStream << "prediction id=" << id;
        std::string title = stringStream.str();
        // utils::print_trajectory(title, trajectory);
    }
*/

    // chooses the best trajectory for given successor states.
    for (auto it = sstates.begin(); it != sstates.end(); ++it)
    {
        string s = *it;
        vector<Vehicle> trajectory = generate_trajectory(self, s, predictions);
        if (!trajectory.empty())
        {
            // utils::print_trajectory(s, trajectory);
            // std::cout << "plan: building target from trajectory for state=" << s << std::endl;
            // std::cout << "plan: self_car={s=" << self.s << ", d=" << self.d << ", speed=" << self.speed << std::endl;
            Target target = build_target(trajectory, predictions);
            double cost = calculate_cost(self, target, trajectory, predictions);
            std::cout << "plan: ======== " << s << ": cost=" << cost << std::endl;
            // std::cout << "plan: ======== " << s << ": target speed=" << target.speed << ", lane=" << target.lane << std::endl;
            if (cost < min_cost)
            {
                min_cost = cost;
                best_target = target;
                best_state = s;
            }
        }
    }

    this->state = best_state;
    std::cout << "The best next state=" << best_state << std::endl;
    return best_target;
};

// call the respective method to generate a trajectory
vector<Vehicle> Behavior::generate_trajectory(const Vehicle &self, const string &state, const map<int, vector<Vehicle>>& predictions)
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

vector<Vehicle> Behavior::keep_lane_trajectory(const Vehicle &self, const map<int, vector<Vehicle>>& predictions)
{
    int self_lane = road.get_lane(self.d);
    // std::cout << "keep_lane_trajectory: self_lane=" << self_lane << std::endl;
    return rough_trajectory(self, predictions, self_lane);
}

vector<Vehicle> Behavior::change_lane_left_trajectory(const Vehicle &self, const map<int, vector<Vehicle>>& predictions)
{
    int self_lane = road.get_lane(self.d);

    // cannot change lane as the vehicle is on the most left lane.
    if (!road.can_change_lane_left(self_lane)) {
        // std::cout << "change_lane_left_trajectory: cannot change lane: self_lane=" << self_lane << std::endl;
        return {};
    }

    return rough_trajectory(self, predictions, self_lane - 1);
}

vector<Vehicle> Behavior::change_lane_right_trajectory(const Vehicle &self, const map<int, vector<Vehicle>>& predictions)
{
    int self_lane = road.get_lane(self.d);

    // cannot change lane as the vehicle is on the most right lane.
    if (!road.can_change_lane_right(self_lane)) {
        // std::cout << "change_lane_right_trajectory: cannot change lane: self_lane=" << self_lane << std::endl;
        return {};
    }

    return rough_trajectory(self, predictions, self_lane + 1);
}

vector<Vehicle> Behavior::rough_trajectory(const Vehicle& self, const map<int, vector<Vehicle>>& predictions, int target_lane) 
{
    // initial state
    double target_d = road.lane_center(target_lane);
    double car_d = self.d;
    double car_s = self.s;
    double car_v = MPH2mps(self.speed); // convert to velocity in meters per second
    double dt = pred_resolution_sec; // time between iterations

    // std::cout << "rough_trajectory: target_d=" << target_d << std::endl;

    vector<Vehicle> trajectory;
    trajectory.push_back(self); // always put the current state in the beginning

    int num_iter = pred_horizon_sec / pred_resolution_sec;
    for(int i=0; i<num_iter-1; i++)
    {
        // std::cout << "rough_trajectory: iter=" << i+1 << " car={s=" << car_s << ", d=" << car_d << ", v=" << car_v << "}" << std::endl;

        // finding max and min safe velocity
        double car_v_max = car_v + dt*max_accel;
        double car_v_min = car_v - dt*max_accel;

        // std::cout << "rough_trajectory: iter=" << i+1 << " car_v_max=" << car_v_max << ", velocity_limit=" << velocity_limit << std::endl;

        // the ego-car always tries to increase the speed, but never exceed the speed limit
        car_v = min(car_v_max, velocity_limit);

        // finds the fron center vehicle and slows down if there's any.
        // this is in order to not crash.
        Vehicle vehicle_ahead;
        int vehicle_ahead_id;
        if (get_vehicle_ahead(i, car_s, car_d, predictions, &vehicle_ahead_id, vehicle_ahead)) 
        {
            double first_s = predictions.at(vehicle_ahead_id).at(0).s;
            double first_d = predictions.at(vehicle_ahead_id).at(0).d;
    
            bool vehicle_was_behind = first_s < self.s;
            bool vehicle_in_same_lane = road.get_lane(first_d) == road.get_lane(self.d);

            // ignore vehicles which are right behind the ego-car at the begining.
            if (vehicle_was_behind && vehicle_in_same_lane) 
            {
                std::cout << "rough_trajectory: iter=" << i+1 << " ignore vehicle id=" << vehicle_ahead_id << " as it's right behind self car" << std::endl;
            }
            else 
            {
                // finds if we need to slow down and with with speed.
                double ds = vehicle_ahead.s - car_s;
                double vehicle_ahead_vel = MPH2mps(vehicle_ahead.speed);
                double max_velocity_in_front = (ds - preferred_buffer - Vehicle::LENGTH)/dt + vehicle_ahead_vel - 0.5*max_accel;
                // std::cout << "rough_trajectory: iter=" << i+1 << " max_velocity_in_front=" << max_velocity_in_front << ", car_v=" << car_v << std::endl;

                // limits the minimum velocitu in order to not exceed the max acceleration
                car_v = max(
                    min(max_velocity_in_front, car_v),
                    car_v_min
                );
            }
        }

        // calculating the next position
        double dist = car_v * dt; // the distance the ego-car will move in time dt going with velocity car_v
        double max_angle = 45; // degrees
        double max_dd = dist * cos(deg2rad(max_angle)); // find the maximum d offset
        double dd = target_d - car_d;

        // prevent moving horizontally too much
        if (dd > max_dd) dd = max_dd;
        else if (dd < -max_dd) dd = -max_dd;
        
        // find the s-position increment
        double ds = sqrt(dist*dist - dd*dd);

        // std::cout << "dist=" << dist << ", max_dd=" << max_dd << ", dd=" << dd << ", ds=" << ds << std::endl;

        car_d += dd;
        car_s += ds;

        auto xy = road.get_xy(car_s, car_d);
        double speed = utils::mps2MPH(car_v);
        auto v = Vehicle(xy[0], xy[1], car_s, car_d, self.yaw, speed);
        trajectory.push_back(v);
    }

    return trajectory;    
}

bool Behavior::get_vehicle_ahead(int iter, double car_s, double car_d, const map<int, vector<Vehicle>>& predictions, int* id, Vehicle& vehicle_ahead) 
{
    bool found = false;
    double distance = 9999999999;
    for(auto it=predictions.begin(); it != predictions.end(); ++it)
    {
        auto v = it->second.at(iter);
        double dd = abs(v.d - car_d);
        double ds = v.s - car_s;
        bool is_further = ds > 0;
        bool is_near_d = dd <= Vehicle::WIDTH*1.1;
        if (is_further && is_near_d) 
        {
            if (ds < distance) {
                // this finds the closest front center vehicle
                distance = ds;
                *id = it->first;
                vehicle_ahead = v;
                found = true;
            }
        }
    }
    return found;
}

double Behavior::calculate_cost(const Vehicle &self, const Target& target, const vector<Vehicle> &trajectory, const map<int, vector<Vehicle>>& predictions)
{
    // weights for the respective cost functions
    double w_collision = 100;
    double w_inefficiency = 10;
    double w_change_lane = 1;

    vector<string> cf_name_list
    = {"collision_cost", "inefficiency_cost", "change_lane_cost"};

    vector< function<double (const cost::cost_context&)>> cf_list 
    = {cost::collision_cost, cost::inefficiency_cost, cost::change_lane_cost};

    vector<double> weight_list = {w_collision, w_inefficiency, w_change_lane};

    cost::cost_context ctx;
    ctx.road = &road;
    ctx.target = target;
    ctx.trajectory = trajectory;
    ctx.predictions = predictions;

    double cost = 0;
    for(int i=0; i<weight_list.size(); i++) {
        string name = cf_name_list[i];
        double w = weight_list[i];
        auto cf = cf_list[i];
        double c = cf(ctx);
        double weighted_cost = w * c;  
        cost += weighted_cost;
        // std::cout << name << ": W=" << w << ", cost=" << c << ", weighed_cost=" << weighted_cost << std::endl;
    } 

    return cost;
}

Target Behavior::build_target(const vector<Vehicle> &trajectory, const map<int, vector<Vehicle>>& predictions)
{
    int last_idx = trajectory.size() - 1;

    Vehicle last_state = trajectory[last_idx];
    Vehicle self = trajectory[0];

    Target t;
    t.lane = road.get_lane(last_state.d);
    t.speed = last_state.speed;
    t.vehicle_id = -1;

    int veh_ahead_id;
    Vehicle veh_ahead;
    if (get_vehicle_ahead(0, self.s, last_state.d, predictions, &veh_ahead_id, veh_ahead)) 
    {
        double dist = utils::distance(self.s, self.d, veh_ahead.s, veh_ahead.d);
        double max_sensor_range = 45;
        // if there is a front center car closer than 45 meters
        // we need to follow the car
        if (dist <= max_sensor_range)
        {
            t.vehicle_id = veh_ahead_id;
            t.speed = -1;
        }
    }

    // std::cout << "Build target for vehicle(d=" << last_state.d << "): t.lane=" << t.lane << ", t.speed=" << t.speed << ", t.vehicle_id=" << t.vehicle_id << std::endl;
    assert(t.lane != -1);

    return t;
}