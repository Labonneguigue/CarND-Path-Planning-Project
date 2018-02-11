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
    void updateCarsData(std::vector<std::vector<double>>& sensordata);

    /** Initialisation and update of myAV data.
     *
     */
    void updateMyAVData(double x_,
                        double y_,
                        double s_,
                        double d_,
                        double yaw_,
                        double speed_);

    /** Returns a reference to myAV
     *
     */
    VehicleData& myAV()
    {
        return mMyAV;
    }

    /** Detects whether there is a car ahead or not, if it is the case
     *  the distance to that car is returned in the variable distance as
     *  well as its speed. The closest car is returned if multiple cars are
     *  detected ahead of us in the current lane.
     *
     * @note The number of the lane of myAV and its s Frenet coordinate are used
     *       to compute the following results
     *
     * @param[out] distance Distance to the car in front. max double if no car found
     * @param[out] speed Speed to the car in front. max double if no car found
     *
     * @return True if vehicle is ahead and the data in distance is correct, False otherwise
     */
    bool getDistanceAndSpeedCarAhead(double& distance, double& speed);

private:

    std::vector<DetectedVehicleData> mCars; ///< Vector of data about detected cars

    VehicleData mMyAV; ///< Instance of my autonomous vehicle
};

#endif // SENSOR_FUSION_H