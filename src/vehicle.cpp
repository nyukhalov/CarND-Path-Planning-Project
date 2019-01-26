#include "vehicle.h"
#include "road.h"

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed) {
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;
}

const int Vehicle::get_lane() const {
    return road::get_lane(d);
}