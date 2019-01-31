#include "road.h"
#include "utils.h"

Road::Road(int num_lanes, double lane_width, const vector<double>& wps_x, const vector<double>& wps_y, 
    const vector<double>& wps_s, const vector<double>& wps_dx, const vector<double>& wps_dy)
: lane_width(lane_width)
{
    this->num_lanes = num_lanes;
    this->wps_x = wps_x;
    this->wps_y = wps_y;
    this->wps_s = wps_s;
    this->wps_dx = wps_dx;
    this->wps_dy = wps_dy;
}

double Road::lane_center(int lane) const
{
    return (lane_width * 0.5) + (lane_width * lane);
}

int Road::get_lane(double d) const
{
    for (int lane = 0; lane < num_lanes; lane++)
    {
        if (is_within_lane(lane, d))
            return lane;
    }
    return -1;
}

bool Road::is_within_lane(int lane, double d) const
{
    double lane_boundary_left = lane_width * lane;
    double lane_boundary_right = lane_width * (lane + 1);
    return lane_boundary_left <= d && d < lane_boundary_right;
}

vector<double> Road::get_xy(double s, double d) const 
{
    return utils::getXY(s, d, wps_s, wps_x, wps_y);
}

vector<double> Road::get_frenet(double x, double y, double theta) const {
    return utils::getFrenet(x, y, theta, wps_x, wps_y);
}

bool Road::can_change_lane_right(int lane) const
{
    return lane < num_lanes - 1;
}

bool Road::can_change_lane_left(int lane) const
{
    return lane > 0;
}