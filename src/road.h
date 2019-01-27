#ifndef _ROAD_H_
#define _ROAD_H_

#include <vector>

using namespace std;

class Road
{
public:
    Road(int num_lanes, double lane_width, const vector<double>& wps_x, const vector<double>& wps_y, 
    const vector<double>& wps_s, const vector<double>& wps_dx, const vector<double>& wps_dy);

    double lane_center(int lane) const;

    int get_lane(double d) const;

    bool is_within_lane(int lane, double d) const;

    vector<double> get_xy(double s, double d) const;

    vector<double> get_frenet(double x, double y, double theta) const;

private:
    double lane_width;
    int num_lanes;
    vector<double> wps_x;
    vector<double> wps_y;
    vector<double> wps_s;
    vector<double> wps_dx;
    vector<double> wps_dy;
};

#endif // _ROAD_H_