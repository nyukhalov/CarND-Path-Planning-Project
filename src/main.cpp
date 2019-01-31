#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "road.h"
#include "fsm.h"
#include "behavior.h"
#include "prediction.h"
#include "vehicle.h"
#include "spline.h"
#include "utils.h"

using namespace std;
using namespace utils;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != string::npos)
	{
		return "";
	}
	else if (b1 != string::npos && b2 != string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main()
{
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	// The max s value before wrapping around the track back to 0
	double max_s = 6945.554;

	ifstream in_map_(map_file_.c_str(), ifstream::in);

	string line;
	while (getline(in_map_, line))
	{
		istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	long iteration = 0L;

	double update_freq_sec = 0.02;
	int num_lanes = 3;
	double lane_width = 4.0;
	double speed_limit = 49.5; // Miles per hour
	double pred_horizon_sec = 5.0;
	double pred_resolution_sec = update_freq_sec; // the same as update frequency (50 updates / sec)
	double behaviour_interval_sec = 0.5;
	int behaviour_interval_iter = (int) (behaviour_interval_sec / update_freq_sec);

	double cur_velocity = 0.0;
	double velocity_limit = utils::MPH2mps(speed_limit);
	double max_accel = 9.0;
	double dist_fc = 0.0;

	Road road(num_lanes, lane_width, map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
	Prediction prediction(road, pred_horizon_sec, pred_resolution_sec);
	Behavior behavior(pred_horizon_sec, pred_resolution_sec, behaviour_interval_sec, speed_limit, max_accel, road);	

	Target target;

	h.onMessage([&road, &prediction, &behavior, &iteration, &target, &cur_velocity, &dist_fc, max_accel, velocity_limit, behaviour_interval_iter, update_freq_sec](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = j[1]["speed"];

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					auto sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					vector<double> next_x_vals;
					vector<double> next_y_vals;

					iteration++;

					Vehicle self(car_x, car_y, car_s, car_d, car_yaw, car_speed);
					auto predictions = prediction.predict(sensor_fusion);
					if (iteration <= 1 || iteration % behaviour_interval_iter == 0) {
						std::cout << std::endl << "Iteration=" << iteration << ". Trigger bahavior planner" << std::endl;
						Target t = behavior.plan(self, predictions);
						target = t;
						std::cout << "Target: speed=" << target.speed << ", lane=" << target.lane << ", vehicle_id=" << target.vehicle_id << std::endl;
					}

					int prev_path_size = previous_path_x.size();

					double last_car_x, last_car_y; // car's position at the end of its last traejctory
					double prev_car_x, prev_car_y; // one point before
					double ref_car_yaw_rad;

					if (prev_path_size >= 2)
					{
						last_car_x = previous_path_x[prev_path_size - 1];
						last_car_y = previous_path_y[prev_path_size - 1];

						prev_car_x = previous_path_x[prev_path_size - 2];
						prev_car_y = previous_path_y[prev_path_size - 2];

						ref_car_yaw_rad = atan2(last_car_y - prev_car_y, last_car_x - prev_car_x);
					}
					else
					{
						ref_car_yaw_rad = deg2rad(car_yaw);

						last_car_x = car_x;
						last_car_y = car_y;

						prev_car_x = last_car_x - cos(ref_car_yaw_rad);
						prev_car_y = last_car_y - sin(ref_car_yaw_rad);
					}
					
					double wp_ahead = min(
						35.0, 
						max(30.0, 2.0 * cur_velocity)
					);
					double wp_d = road.lane_center(target.lane);
					vector<double> next_wp1 = road.get_xy(car_s + wp_ahead, wp_d);
					vector<double> next_wp2 = road.get_xy(car_s + wp_ahead + 30, wp_d);
					vector<double> next_wp3 = road.get_xy(car_s + wp_ahead + 60, wp_d);

					vector<double> traj_keypoints_x;
					vector<double> traj_keypoints_y;

					traj_keypoints_x.push_back(prev_car_x);
					traj_keypoints_x.push_back(last_car_x);
					traj_keypoints_x.push_back(next_wp1[0]);
					traj_keypoints_x.push_back(next_wp2[0]);
					traj_keypoints_x.push_back(next_wp3[0]);

					traj_keypoints_y.push_back(prev_car_y);
					traj_keypoints_y.push_back(last_car_y);
					traj_keypoints_y.push_back(next_wp1[1]);
					traj_keypoints_y.push_back(next_wp2[1]);
					traj_keypoints_y.push_back(next_wp3[1]);

					// rotating
					for (int i=0; i<traj_keypoints_x.size(); i++)
					{
						double shift_car_x = traj_keypoints_x[i] - last_car_x;
						double shift_car_y = traj_keypoints_y[i] - last_car_y;

						traj_keypoints_x[i] = shift_car_x * cos(0-ref_car_yaw_rad) - shift_car_y * sin(0-ref_car_yaw_rad);
						traj_keypoints_y[i] = shift_car_x * sin(0-ref_car_yaw_rad) + shift_car_y * cos(0-ref_car_yaw_rad);
					}

					tk::spline s;
					// currently it is required that X is already sorted
					s.set_points(traj_keypoints_x, traj_keypoints_y);

					int points_ahead = 10;

					for (int i = 0; i < previous_path_x.size(); i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt(target_x*target_x + target_y*target_y);

					double target_vel = utils::MPH2mps(target.speed);
					double cur_car_x = last_car_x;
					double cur_car_y = last_car_y;
					double x_acc = 0;
					for (int i=0; i<points_ahead-previous_path_x.size(); i++)
					{
						int iter = previous_path_x.size() + i;
						if (target.vehicle_id >= 0) 
						{
							auto traj = predictions.at(target.vehicle_id);
							Vehicle veh = traj[iter];
							// dist_fc = distance from bamper to bamper
							double new_dist_fc = utils::distance(cur_car_x, cur_car_y, veh.x, veh.y) - Vehicle::LENGTH;
							if (dist_fc == 0) dist_fc = new_dist_fc;
							double v_fc = (new_dist_fc - dist_fc) / update_freq_sec;
							dist_fc = new_dist_fc;
							double buffer_fc = 8.0;
							double Kp = 0.5;
							double Kv = 1;

							double dist_to_go = dist_fc-buffer_fc;
							double accel = Kp*dist_to_go + Kv*v_fc;
							target_vel = cur_velocity + update_freq_sec*accel;

							// std::cout << "iter=" << iter << ", dist_to_go=" << dist_to_go << ", dist_fc=" << dist_fc << ", v_fc=" << v_fc << ", accel=" << accel << ", target_vel=" << target_vel << std::endl;
						} 
						else
						{
							dist_fc = 0.0;
						}

						double max_vel = cur_velocity + update_freq_sec*max_accel;
						double min_vel = cur_velocity - update_freq_sec*max_accel;
						double vel = min(
							min(max_vel, target_vel),
							velocity_limit	
						);
						vel = max(vel, min_vel);
						cur_velocity = vel;

						double N = target_dist / (update_freq_sec * vel);
						x_acc += target_x / N;
						double y = s(x_acc);

						// rotate back to normal
						double x_norm = x_acc*cos(ref_car_yaw_rad) - y*sin(ref_car_yaw_rad);
						double y_norm = x_acc*sin(ref_car_yaw_rad) + y*cos(ref_car_yaw_rad);

						cur_car_x = x_norm + last_car_x;
						cur_car_y = y_norm + last_car_y;

						next_x_vals.push_back(cur_car_x);
						next_y_vals.push_back(cur_car_y);
					}

					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
