#ifndef _VEHICLE_H_
#define _VEHICLE_H_

class Vehicle {
public:
    Vehicle(double x, double y, double s, double d, double yaw, double speed);

    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

#endif //_VEHICLE_H_