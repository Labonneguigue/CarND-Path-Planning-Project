#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>
#include "vehicledata.h"
#include "utl.h"

class SensorFusion
{
public:

    /** Default constructor
     *
     */
    SensorFusion();

    /** Default destructor
     *
     */
    ~SensorFusion();

    /** Initialisation and update using the data provided by the perception module
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
    void updateData(std::vector<std::vector<double>>& sensordata);

    /** Detects whether there is a car ahead or not, if it is the case
     *  the distance to that car is returned in the variable distance as
     *  well as its speed. The closest car is returned if multiple cars are
     *  detected ahead of us in the current lane.
     *
     * @param[in] laneNumber Number of the lane to be checked (current vehicle lane)
     * @param[in} current_s Frenet s coordinate of the car
     * @param[out] distance Distance to the car in front. max double if no car found
     * @param[out] speed Speed to the car in front. max double if no car found
     *
     * @return True if vehicle is ahead and the data in distance is correct, False otherwise
     */
    bool getDistanceAndSpeedCarAhead(double laneNumber, double current_s, double& distance, double& speed);

private:

    std::vector<DetectedVehicleData> cars;
};

#endif // SENSOR_FUSION_H
