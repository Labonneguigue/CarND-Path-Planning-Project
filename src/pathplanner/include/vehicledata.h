#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <vector>

///< @todo: Should be classes since data members are interdependant

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

struct DetectedVehicleData : public VehicleData
{
    double id;
    double x_dot;
    double y_dot;
    double timeSinceLastUpdate;

    DetectedVehicleData(double id,
                        double x,
                        double y,
                        double x_dot,
                        double y_dot,
                        double s,
                        double d)
    : VehicleData(x, y, s, d, 0, 0)
    , id(id)
    , x_dot(x_dot)
    , y_dot(y_dot)
    {}

    void timeElapsed()
    {
        timeSinceLastUpdate++;
    }

    void updateData(double x_,
                    double y_,
                    double x_dot_,
                    double y_dot_,
                    double s_,
                    double d_)
    {
        x = x_;
        y = y_;
        x_dot = x_dot_;
        y_dot = y_dot_;
        s = s_;
        d = d_;
        timeSinceLastUpdate = 0;
    }


    /** Implementation of the operator< to enable sorting
     *
     */
    bool operator < (const DetectedVehicleData& car) const
    {
        return (id < car.id);
    }

};

#endif // VEHICLE_DATA_H
