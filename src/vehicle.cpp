#include "vehicle.h"
#include "road.h"

Vehicle::Vehicle() {}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed) {
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;
}