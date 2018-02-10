#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <vector>

struct VehicleData
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;

    VehicleData(double x, double y, double s, double d, double yaw, double speed)
    : x(x)
    , y(y)
    , s(s)
    , d(d)
    , yaw(yaw)
    , speed(speed)
    {}
    
};

#endif // VEHICLE_DATA_H
