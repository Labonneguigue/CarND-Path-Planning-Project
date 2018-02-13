#include <limits>
#include <cassert>
#include "sensorfusion.h"
#include "drivingpolicy.h"

SensorFusion::SensorFusion()
: mCars()
, mMyAV()
, mHighway(policy::defaultNbLanes)
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
                                                sensordata[car][6],
                                                getVehicleLane(sensordata[car][6])));
        }
        std::sort(mCars.begin(), mCars.end());
    }
    else
    {
        for (int newData = 0 ; newData < sensordata.size() ; ++newData)
        {
            bool found = false;
            // Since the previous records are sorted, I iterate until
            for (int previousRecord = 0 ; ((mCars[previousRecord].id <= sensordata[newData][0]) &&
                                           (previousRecord < mCars.size()) &&
                                           (!found)) ; ++previousRecord)
            {
                if (mCars[previousRecord].id == sensordata[newData][0])
                {
                    //Record exist, I update it.
                    mCars[previousRecord].updateData(sensordata[newData][1],
                                                     sensordata[newData][2],
                                                     sensordata[newData][3],
                                                     sensordata[newData][4],
                                                     sensordata[newData][5],
                                                     sensordata[newData][6],
                                                     getVehicleLane(sensordata[newData][6]));
                    //
                    found = true;
                }
            }
            if (!found)
            {
                //Record doesn't exist, I append it at the end
                mCars.push_back(DetectedVehicleData(sensordata[newData][0],
                                                    sensordata[newData][1],
                                                    sensordata[newData][2],
                                                    sensordata[newData][3],
                                                    sensordata[newData][4],
                                                    sensordata[newData][5],
                                                    sensordata[newData][6],
                                                    getVehicleLane(sensordata[newData][6])));

            }
        }
        // Finish merging, I sort it for nex time
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
        if (utl::isCarInLane<double>(mMyAV.lane, mCars[car].d) &&
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

Lane SensorFusion::getVehicleLane(const double d) const
{
    for (int lane = 0; lane < mHighway.getNumberLanes() ; ++lane)
    {
        if (utl::isCarInLane(lane, d))
        {
            return static_cast<Lane>(lane);
        }
    }
    return undefined;
}
