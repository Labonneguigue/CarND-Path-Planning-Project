#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>
#include "vehicledata.h"

struct SensorFusion
{
    std::vector<DetectedVehicleData> cars;

    /** Constructor using the data provided by the perception module
     *
     * @param sensordata Data about each car detected around our vehicle
     * @note The content of the nested vector<double> is as follow:
     *       0 - car's unique ID
     *       1 - car's x position in map coordinates
     *       2 - car's y position in map coordinates
     *       3 - car's x velocity in m/s
     *       4 - car's y velocity in m/s
     *       5 - car's s position in frenet coordinates
     *       6 - car's d position in frenet coordinates
     */
    SensorFusion(std::vector<std::vector<double>>& sensordata)
    {
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
    }


    bool getDistanceCarAhead(double laneNumber, double& distance)
    {
        for (int car = 0; car++)
    }
};

#endif // SENSOR_FUSION_H
