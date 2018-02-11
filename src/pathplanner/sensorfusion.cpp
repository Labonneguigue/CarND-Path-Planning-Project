#include <limits>
#include <cassert>
#include "sensorfusion.h"

SensorFusion::SensorFusion()
: mCars()
, mMyAV()
{}

SensorFusion::~SensorFusion()
{}

void SensorFusion::updateCarsData(std::vector<std::vector<double>>& sensordata)
{
    if (mCars.empty()){
        for (int car = 0 ; car < sensordata.size() ; ++car)
        {
            mCars.push_back(DetectedVehicleData(sensordata[car][0],
                                               sensordata[car][1],
                                               sensordata[car][2],
                                               sensordata[car][3],
                                               sensordata[car][4],
                                               sensordata[car][5],
                                               sensordata[car][6]));
        }
        std::sort(mCars.begin(), mCars.end());
    }
    else
    {
        /// @todo: Merge new data with previous if it makes sense
        /// For now, it just overrides everything
        mCars.clear();
        for (int car = 0 ; car < sensordata.size() ; ++car)
        {
            mCars.push_back(DetectedVehicleData(sensordata[car][0],
                                               sensordata[car][1],
                                               sensordata[car][2],
                                               sensordata[car][3],
                                               sensordata[car][4],
                                               sensordata[car][5],
                                               sensordata[car][6]));
        }
        std::sort(mCars.begin(), mCars.end());
    }
}

void SensorFusion::updateMyAVData(double x_,
                                  double y_,
                                  double s_,
                                  double d_,
                                  double yaw_,
                                  double speed_)
{
    mMyAV.updateData(x_, y_, s_, d_, yaw_, speed_);
}


bool SensorFusion::getDistanceAndSpeedCarAhead(double& distance,
                                               double& speed)
{
    bool found = false;
    distance = std::numeric_limits<double>::max();
    speed = std::numeric_limits<double>::max();
    for (int car = 0; car < mCars.size() ; car++)
    {
        // If the car is in my lane and its s is greater than mine,
        // I consider it
        if (utl::isCarInMyLane<double>(mMyAV.lane, mCars[car].d) &&
           (mCars[car].s > mMyAV.s))
        {
            // If the distance between us is smaller that previously
            // recorded, it becomes the closest car to me.
            if ((mCars[car].s - mMyAV.s) < distance)
            {
                distance = mCars[car].s - mMyAV.s;
                speed = sqrt(utl::sqr(mCars[car].x_dot) + utl::sqr(mCars[car].y_dot));
                found = true;
                assert(distance > 0);
            }
        }
    }
    return found;
}


