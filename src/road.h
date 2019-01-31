#ifndef _ROAD_H_
#define _ROAD_H_

#include <vector>

using namespace std;

// Defines a road and provides several useful methods.
class Road
{
public:
    Road(int num_lanes, double lane_width, const vector<double>& wps_x, const vector<double>& wps_y, 
    const vector<double>& wps_s, const vector<double>& wps_dx, const vector<double>& wps_dy);

    // returns the given lane's center.
    double lane_center(int lane) const;

    // returns the lane number for the given d or -1 if d is out of valid range.
    int get_lane(double d) const;

    // returns true if the given d is within the given lane, false otherwise.
    bool is_within_lane(int lane, double d) const;

    // converts the given (s,d) to (x,y) coordinates.
    vector<double> get_xy(double s, double d) const;

    // converts the given (x,y,t) to (s,d) coordinates.
    vector<double> get_frenet(double x, double y, double theta) const;

    // returns true if there's another lane on the right of the given lane.
    bool can_change_lane_right(int lane) const;

    // returns true if there's another lane on the left of the given lane.
    bool can_change_lane_left(int lane) const;

    const double lane_width;

private:
    int num_lanes;
    vector<double> wps_x;
    vector<double> wps_y;
    vector<double> wps_s;
    vector<double> wps_dx;
    vector<double> wps_dy;
};

#endif // _ROAD_H_