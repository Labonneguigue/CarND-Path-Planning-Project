#include <limits>
#include <cassert>
#include <algorithm>
#include "sensorfusion.h"
#include "drivingpolicy.h"

#define VERBOSE 1

SensorFusion::SensorFusion()
: mCars()
, mCarAhead(nullptr)
, mMyAV()
, mHighway(policy::defaultNbLanes)
{}

SensorFusion::~SensorFusion()
{}

void SensorFusion::updateCarsData(std::vector<std::vector<double>>& sensordata)
{
    if (mCars.empty()){
        for (unsigned int car = 0 ; car < sensordata.size() ; ++car)
        {
            mCars.push_back(DetectedVehicleData(sensordata[car][0], // id
                                                sensordata[car][1], // x
                                                sensordata[car][2], // y
                                                sensordata[car][3], // x_dot
                                                sensordata[car][4], // y_dot
                                                sensordata[car][5], // s
                                                sensordata[car][6], // d
                                                getVehicleLane<Lane>(sensordata[car][6])));
        }
        std::sort(mCars.begin(), mCars.end(), [](const DetectedVehicleData& lhs, const DetectedVehicleData& rhs)
        {
            return lhs.id < rhs.id;
        });
    }
    else
    {
        for (unsigned int newData = 0 ; newData < sensordata.size() ; ++newData)
        {
            bool found = false;
            // Since the previous records are sorted, I don't have to iterate through the whole set of cars.
            for (unsigned int previousRecord = 0 ;
                 ((mCars[previousRecord].id <= sensordata[newData][0]) &&
                  (previousRecord < mCars.size()) && (!found)) ; ++previousRecord)
            {
                if (mCars[previousRecord].id == sensordata[newData][0])
                {
                    // Record exist, I update it.
                    mCars[previousRecord].updateData(sensordata[newData][1],
                                                     sensordata[newData][2],
                                                     sensordata[newData][3],
                                                     sensordata[newData][4],
                                                     sensordata[newData][5],
                                                     sensordata[newData][6],
                                                     getVehicleLane<Lane>(sensordata[newData][6]));
                    found = true;
                }
            }
            if (!found)
            {
                // Record doesn't exist, I append it at the end
                mCars.push_back(DetectedVehicleData(sensordata[newData][0],
                                                    sensordata[newData][1],
                                                    sensordata[newData][2],
                                                    sensordata[newData][3],
                                                    sensordata[newData][4],
                                                    sensordata[newData][5],
                                                    sensordata[newData][6],
                                                    getVehicleLane<Lane>(sensordata[newData][6])));
            }
        }
        // Finish merging, I sort it for next time
        //std::sort(mCars.begin(), mCars.end());
        std::sort(mCars.begin(), mCars.end(), [](const DetectedVehicleData& lhs, const DetectedVehicleData& rhs)
                  {
                      return lhs.id < rhs.id;
                  });
    }
}

void SensorFusion::updateMyAVData(double x_,
                                  double y_,
                                  double s_,
                                  double d_,
                                  double yaw_,
                                  double speedmph)
{
    const double speedms = utl::mph2ms(speedmph);
    mMyAV.updateData(x_, y_, s_, d_, yaw_, speedms, getVehicleLane<Lane>(d_));
}

const std::vector<DetectedVehicleData> SensorFusion::detectedCars() const
{
    std::vector<DetectedVehicleData> recentlyDetectedCars;
    for (const DetectedVehicleData car : mCars)
    {
        if (car.hasBeenUpdatedRecently())
        {
            recentlyDetectedCars.push_back(car);
        }
    }
#if VERBOSE > 2
    std::cout << "Size of recently detected cars : " << recentlyDetectedCars.size() << "\n";
#endif
    return recentlyDetectedCars;
}
