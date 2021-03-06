#ifndef VEHICLE_DATA_H
#define VEHICLE_DATA_H

#include <iostream>
#include <chrono>
#include <vector>
#include "roadtypes.h" // Lane
#include "utl.h"

///< @todo: Should be classes since data members are interdependant

struct VehicleData
{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speedMs; ///< Effective car speed [m/s]
    Lane lane; ///< Enum keeping track of the lane myAV is in

    /** Default constructor
     *
     */
    VehicleData()
    : x(0)
    , y(0)
    , s(0)
    , d(0)
    , yaw()
    , speedMs()
    , lane(secondLane)
    {}

    /** Constructor with non zero fields initialisation
     *
     */
    VehicleData(double x_,
                double y_,
                double s_,
                double d_,
                double yaw_,
                double speedMs_,
                Lane lane_ = secondLane)
    : x(x_)
    , y(y_)
    , s(s_)
    , d(d_)
    , yaw(yaw_)
    , speedMs(speedMs_)
    , lane(lane_)
    {}

    /** Update Data
     *
     */
    void updateData(double x_,
                    double y_,
                    double s_,
                    double d_,
                    double yaw_,
                    double speedMs_,
                    Lane lane_ = undefined)
    {
        x = x_;
        y = y_;
        s = s_;
        d = d_;
        yaw = yaw_;
        speedMs = speedMs_;
        if (lane_ != undefined)
        {
            //If not, that means I'm probably switching lanes
            lane = lane_;
        }
    }


};


/** DetectedVehicleData structure
 *
 */
struct DetectedVehicleData : public VehicleData
{
    double id;
    double x_dot;
    double y_dot;
    double s_dot;
    double d_dot; ///< Temporal derivative of d gives information on whether the car is changing lane or not
    bool isChangingLane; ///< Flag raised when the car is detected to change lane
    std::chrono::high_resolution_clock::time_point lastUpdateTimeStamp; ///< Timestamp set when the struct members are updated 

    /** Constructor
     *
     */
    DetectedVehicleData(double id,
                        double x,
                        double y,
                        double x_dot,
                        double y_dot,
                        double s,
                        double d,
                        Lane lane_)
    : VehicleData(x, y, s, d, 0.0, utl::distance(x_dot, y_dot, 0.0, 0.0), lane_)
    , id(id)
    , x_dot(x_dot)
    , y_dot(y_dot)
    , s_dot(0.0)
    , d_dot(0.0)
    , isChangingLane(false)
    {
    }

    /** Update the data
     *
     * @note By computing the time derivative of d and s I obtain the longitudinal 
     *      velocity of the car as well as the lateral one. Both being very helpful
     *      when I comes to deciding where to go on the road.
     */
    void updateData(double x_,
                    double y_,
                    double x_dot_,
                    double y_dot_,
                    double s_,
                    double d_,
                    Lane lane_)
    {
        x = x_;
        y = y_;
        x_dot = x_dot_;
        y_dot = y_dot_;
        speedMs = utl::distance(x_dot, y_dot, 0.0, 0.0);
        s_dot = (s_ - s) / (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastUpdateTimeStamp).count() * 1000000);
        s = s_;
        d_dot = (d_ - d) / (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - lastUpdateTimeStamp).count() * 1000000);
        lastUpdateTimeStamp = std::chrono::high_resolution_clock::now();
        d = d_;
        lane = lane_;
        constexpr const static double d_dotThreshold = 0.001;
        if (d_dot > d_dotThreshold)
        {
            // change to ternary operator once threshold fine tuned
#if VERBOSE > 0
            std::cout << " # # # \t Car " << id << " is changing lane.\n";
#endif
            isChangingLane = true;
        }
        else
        {
            isChangingLane = false;
        }
        print();
    }

    /** If the lane of the vehicle is undefined, then it hasn't got
     * accurate data and shouldn't be considered.
     */
    bool hasBeenUpdatedRecently() const
    {
        ///@todo Might need to change that
        return (lane == undefined) ? false : true;
    }

    /** Print out the data of the data
     *
     */
    void print() const
    {
#if VERBOSE > 2
        std::cout<< "Vehicle " << id << " x:" << x << " y:" << y << " speed [m/s]:" << speedMs
                 << " x_dot: " << x_dot << " y_dot" << y_dot << " s:" << s << " d:" << d
                 << " d_dot:" << d_dot << " lane:" << lane << "\n";
#endif
    }

    /** Implementation of the operator< to enable sorting by position
     *  on the road.
     */
    bool operator < (const DetectedVehicleData& car) const
    {
        return (s < car.s);
        //return (id < car.id);
    }

};

#endif // VEHICLE_DATA_H
