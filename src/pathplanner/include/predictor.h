#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "sensorfusion.h"
#include "vehicledata.h"
#include "drivingpolicy.h"

class Predictor
{
public:

    /** Warnings structure allows to skip the behavior sub-component
     *  to be called at each frame if there are nothing to react to.
     *
     */
    struct Warnings
    {
        bool carCrossingPlannedPath; ///< Flag a car which has started crossing our planned path
        int carCrossingPlannedPathId;

        bool slowCarAhead; ///< Flag a car detected in my lane and drives slower than me
        int slowCarAheadId;

        /** Default constructor
         *
         */
        Warnings()
        : carCrossingPlannedPath(false)
        , slowCarAhead(false)
        {}
    };

    /** Constructor
     *
     * @param[in] Reference to a sensorFusion instance
     *
     * @note This sensorFusion instance will be updated with data from
     *       myAV and detected surrounding cars.
     */
    Predictor(SensorFusion& sensorFusion);

    /** Default destructor
     *
     */
    ~Predictor();

    /**
     *
     */
    const bool anyWarnings(Warnings& warnings) const;

    /**
     *
     */
    void prepareSensorDataForPrediction();

    /** Lane speed is calculated as the slowest vehicle in the lane
     *  For vehicles behind me, I consider them only if I don't have
     *  the time to insert myself within the lane and I'd have to wait
     *  and go after them. To do so, I consider our relative speed and
     *  make sure that my insertion would leave a safe distance between
     *  myself and the other cars.
     *
     * @param[in] lane Lane I want to know the speed of
     * @param[out] laneSpeedMs The speed of the lane in meters / second
     * @param[out] timeToInsertionS Time to wait in lane before changing in seconds
     */
    void getLaneSpeedAndTimeToInsertion(const int lane,
                                        double& laneSpeedMs,
                                        double& timeToInsertionS) const;

    /**
     *
     */
    const bool isCarTooFarBehind(const DetectedVehicleData car) const;

    /**
     *
     */
    const bool isCarTooFarAhead(const DetectedVehicleData car) const;


private:

    SensorFusion& mSensorFusion;
    std::vector<DetectedVehicleData> mNearbyCars;

    double mMaximumAccelerationMs = policy::getSafePolicy(policy::maxAccelerationMs); ///< Maximum allowed acceleration in m/s^2 @note 10
};

#endif //PREDICTOR_H
