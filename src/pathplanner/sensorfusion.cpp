#include <limits>
#include <cassert>
#include "sensorfusion.h"

SensorFusion::SensorFusion()
{}

SensorFusion::~SensorFusion()
{}

void SensorFusion::updateData(std::vector<std::vector<double>>& sensordata)
{
    if (cars.empty()){
        for (int car = 0 ; car < sensordata.size() ; ++car)
        {
            cars.push_back(DetectedVehicleData(sensordata[car][0],
                                               sensordata[car][1],
                                               sensordata[car][2],
                                               sensordata[car][3],
                                               sensordata[car][4],
                                               sensordata[car][5],
                                               sensordata[car][6]));
        }
        std::sort(cars.begin(), cars.end());
    }
    else
    {
        /// @todo: Merge new data with previous if it makes sense
        /// For now, it just overrides everything
        cars.clear();
        for (int car = 0 ; car < sensordata.size() ; ++car)
        {
            cars.push_back(DetectedVehicleData(sensordata[car][0],
                                               sensordata[car][1],
                                               sensordata[car][2],
                                               sensordata[car][3],
                                               sensordata[car][4],
                                               sensordata[car][5],
                                               sensordata[car][6]));
        }
        std::sort(cars.begin(), cars.end());
    }

}

bool SensorFusion::getDistanceAndSpeedCarAhead(double laneNumber,
                                               double current_s,
                                               double& distance,
                                               double& speed)
{
    bool found = false;
    distance = std::numeric_limits<double>::max();
    speed = std::numeric_limits<double>::max();
    for (int car = 0; car < cars.size() ; car++)
    {
        // If the car is in my lane and its s is greater than mine,
        // I consider it
        if (utl::isCarInMyLane(laneNumber, cars[car].d) &&
           (cars[car].s > current_s))
        {
            // If the distance between us is smaller that previously
            // recorded, it becomes the closest car to me.
            if ((cars[car].s - current_s) < distance)
            {
                distance = cars[car].s - current_s;
                speed = sqrt(utl::sqr(cars[car].x_dot) + utl::sqr(cars[car].y_dot));
                found = true;
                assert(distance > 0);
            }
        }
    }
    return found;
}


