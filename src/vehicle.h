#ifndef _VEHICLE_H_
#define _VEHICLE_H_

class Vehicle {
public:
    Vehicle();
    Vehicle(double x, double y, double s, double d, double yaw, double speed);

    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed; // vehicle's speed in miles per hour
};

#endif //_VEHICLE_H_