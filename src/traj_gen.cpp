#include "traj_gen.h"
#include "utils.h"
#include "spline.h"
#include <math.h>

TrajectoryGenerator::TrajectoryGenerator(const Road &road, double update_freq_sec, double max_accel, double speed_limit)
    : road(road)
{
    this->update_freq_sec = update_freq_sec;
    this->max_accel = max_accel;
    this->velocity_limit = utils::MPH2mps(speed_limit);
    this->buffer_fc = 10.0;
    this->cur_velocity = 0.0;
    this->dist_fc = 0.0;
}

vector<Vehicle> TrajectoryGenerator::generate(
    const vector<Vehicle> &prev_trajectory, 
    const Vehicle &self, 
    const map<int, vector<Vehicle>>& predictions, 
    Target target)
{
    // get the two last ego-car's positions from the previous trajectory
    auto last_positions = get_last_position(prev_trajectory, self);

    double prev_car_x = last_positions[0];
    double prev_car_y = last_positions[1];
    double last_car_x = last_positions[2];
    double last_car_y = last_positions[3];
    double ref_car_yaw_rad = last_positions[4];

    // generate trajectory waypoints
    auto traj_waypoints = get_trajectory_waypoints(self, {prev_car_x, prev_car_y}, {last_car_x, last_car_y}, target);
    auto traj_waypoints_x = traj_waypoints[0];
    auto traj_waypoints_y = traj_waypoints[1];

    // rotating the waypoints
    for (int i = 0; i < traj_waypoints_x.size(); i++)
    {
        double shift_car_x = traj_waypoints_x[i] - last_car_x;
        double shift_car_y = traj_waypoints_y[i] - last_car_y;

        traj_waypoints_x[i] = shift_car_x * cos(0 - ref_car_yaw_rad) - shift_car_y * sin(0 - ref_car_yaw_rad);
        traj_waypoints_y[i] = shift_car_x * sin(0 - ref_car_yaw_rad) + shift_car_y * cos(0 - ref_car_yaw_rad);
    }

    tk::spline s;
    s.set_points(traj_waypoints_x, traj_waypoints_y);

    // the number of trajectory points (too big value may cause delays in the behavior).
    int points_ahead = 10;
    
    int prev_path_size = prev_trajectory.size();

    // the result trajectory
    vector<Vehicle> trajectory;

    // coping the previous trajectory
    for (int i=0; i<prev_path_size; i++)
    {
        trajectory.push_back(prev_trajectory[i]);
    }

    // generating the rest points_ahead - prev_path_size trajectory points
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double target_vel = utils::MPH2mps(target.speed);
    double cur_car_x = last_car_x;
    double cur_car_y = last_car_y;
    double x_acc = 0;
    for (int i = 0; i < points_ahead - prev_path_size; i++)
    {
        int iter = prev_path_size + i;

        // the behavior module gave as a vehicle to follow
        if (target.vehicle_id >= 0)
        {
            // the vehicle's predicted trajectory
            auto traj = predictions.at(target.vehicle_id);

            // the vehicle's configuration at iterator iter
            Vehicle veh = traj[iter];

            // calculate bamper-to-bamper distance to the front center vehicle
            double new_dist_fc = utils::distance(cur_car_x, cur_car_y, veh.x, veh.y) - Vehicle::LENGTH;
            if (dist_fc == 0) dist_fc = new_dist_fc;

            // v_fc is the rate of changing the distance to the front center vehicle
            double v_fc = (new_dist_fc - dist_fc) / update_freq_sec;

            // updating dist_fc to reuse in the next iteration
            dist_fc = new_dist_fc;

            // parameters for calculating target velocity in order to follow the front center vehicle
            double Kp = 0.5;
            double Kv = 1;

            // calculate target velocity
            double dist_to_go = dist_fc - buffer_fc;
            double accel = Kp * dist_to_go + Kv * v_fc;
            target_vel = cur_velocity + update_freq_sec * accel;
            // std::cout << "iter=" << iter << ", dist_to_go=" << dist_to_go << ", dist_fc=" << dist_fc << ", v_fc=" << v_fc << ", accel=" << accel << ", target_vel=" << target_vel << std::endl;
        }
        else
        {
            // if there's no vehicle to follow, reset the distance
            dist_fc = 0.0;
        }

        // max and min velocity value to not exceed max acceleration
        double max_vel = cur_velocity + update_freq_sec * max_accel;
        double min_vel = cur_velocity - update_freq_sec * max_accel;

        // calculating the new velocity
        double vel = min(
            min(max_vel, target_vel),
            velocity_limit);
        vel = max(vel, min_vel);
        cur_velocity = vel;

        // determine step
        double N = target_dist / (update_freq_sec * vel);
        x_acc += target_x / N;
        double y = s(x_acc);

        // rotate coordinate back to s/d space
        double x_norm = x_acc * cos(ref_car_yaw_rad) - y * sin(ref_car_yaw_rad);
        double y_norm = x_acc * sin(ref_car_yaw_rad) + y * cos(ref_car_yaw_rad);
        cur_car_x = x_norm + last_car_x;
        cur_car_y = y_norm + last_car_y;

        double cur_speed = utils::mps2MPH(cur_velocity);
        auto v = Vehicle(cur_car_x, cur_car_y, -1, -1, -1, cur_speed);
        trajectory.push_back(v);
    }
    return trajectory;
}

vector<double> TrajectoryGenerator::get_last_position(const vector<Vehicle> &prev_trajectory, const Vehicle &self)
{
    double last_car_x, last_car_y; // car's position at the end of its last traejctory
    double prev_car_x, prev_car_y; // one point before
    double ref_car_yaw_rad;

    int prev_path_size = prev_trajectory.size();
    if (prev_path_size >= 2)
    {
        Vehicle last_veh = prev_trajectory.at(prev_path_size - 1);
        last_car_x = last_veh.x;
        last_car_y = last_veh.y;

        Vehicle prev_veh = prev_trajectory.at(prev_path_size - 2);
        prev_car_x = prev_veh.x;
        prev_car_y = prev_veh.y;

        ref_car_yaw_rad = atan2(last_car_y - prev_car_y, last_car_x - prev_car_x);
    }
    else
    {
        ref_car_yaw_rad = utils::deg2rad(self.yaw);

        last_car_x = self.x;
        last_car_y = self.y;

        prev_car_x = last_car_x - cos(ref_car_yaw_rad);
        prev_car_y = last_car_y - sin(ref_car_yaw_rad);
    }

    return {prev_car_x, prev_car_y, last_car_x, last_car_y, ref_car_yaw_rad};
}

vector<vector<double>> TrajectoryGenerator::get_trajectory_waypoints(
    const Vehicle &self, 
    vector<double> prev_car_xy, 
    vector<double> last_car_xy,
    Target target
)
{
    double prev_car_x = prev_car_xy[0];
    double prev_car_y = prev_car_xy[1];
    double last_car_x = last_car_xy[0];
    double last_car_y = last_car_xy[1];

    double wp_ahead = 45;

    double wp_d = road.lane_center(target.lane);
    vector<double> next_wp1 = road.get_xy(self.s + wp_ahead, wp_d);
    vector<double> next_wp2 = road.get_xy(self.s + wp_ahead + 30, wp_d);
    vector<double> next_wp3 = road.get_xy(self.s + wp_ahead + 60, wp_d);

    vector<double> traj_waypoints_x;
    vector<double> traj_waypoints_y;

    traj_waypoints_x.push_back(prev_car_x);
    traj_waypoints_x.push_back(last_car_x);
    traj_waypoints_x.push_back(next_wp1[0]);
    traj_waypoints_x.push_back(next_wp2[0]);
    traj_waypoints_x.push_back(next_wp3[0]);

    traj_waypoints_y.push_back(prev_car_y);
    traj_waypoints_y.push_back(last_car_y);
    traj_waypoints_y.push_back(next_wp1[1]);
    traj_waypoints_y.push_back(next_wp2[1]);
    traj_waypoints_y.push_back(next_wp3[1]);

    return {traj_waypoints_x, traj_waypoints_y};
}