#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <vector>
#include "roadtypes.h" //Highway
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
                        double speedmph);

    /** Returns a reference to myAV
     *
     */
    VehicleData& myAV()
    {
        return mMyAV;
    }

    /** Returns the detected cars updated with the last 500ms
     *
     */
    const std::vector<DetectedVehicleData> detectedCars() const;

    /**
     *
     */
    const Highway& highway() const
    {
        return mHighway;
    }

    /** Returns the lane of a car given its d frenet component
     *
     * @param[in] d Frenet representation d component
     * 
     * @tparam T Type to be returned - Can be typically either int or Lane
     *
     * @return Lane identifying the lane. Returns undefined if fails.
     */
    template <typename T>
    T getVehicleLane(const double d) const
    {
        std::vector<Lane> lanes = mHighway.getAvailableLanes();
        for (unsigned int lane = 0; lane < lanes.size() ; ++lane)
        {
            if (mHighway.isCarInLane(lanes[lane], d))
            {
                return static_cast<T>(lanes[lane]);
            }
        }
        return static_cast<T>(undefined);
    }

private:

    std::vector<DetectedVehicleData> mCars; ///< Vector of data about detected cars

    DetectedVehicleData * mCarAhead; ///< Car immediately ahead of myAV

    VehicleData mMyAV; ///< Instance of my autonomous vehicle

    Highway mHighway; ///< Highway representation
    
};

#endif // SENSOR_FUSION_H
