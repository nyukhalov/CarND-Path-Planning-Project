#ifndef _ROAD_H_
#define _ROAD_H_

namespace road {

    static double WIDTH = 4.0;

    static bool is_within_lane(int lane, double d) {
		double lane_boundary_left = WIDTH * lane;
		double lane_boundary_right = WIDTH * (lane + 1);
        return lane_boundary_left <= d && d < lane_boundary_right;
    }

    static int get_lane(double d) {
        for(int lane=0; lane<3; lane++) {
            if (is_within_lane(lane, d)) return lane;
        }
        return -1;
    }

    static double lane_center(int lane) {
        return (WIDTH * 0.5) + (WIDTH * lane);
    }
}

#endif // _ROAD_H_