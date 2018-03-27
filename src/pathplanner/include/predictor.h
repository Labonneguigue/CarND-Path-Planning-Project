#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "sensorfusion.h"
#include "vehicledata.h"
#include "drivingpolicy.h"

class Predictor
{
public:

    /** Warnings structure allows to skip the call to the behavior planning
     *  module if there is nothing to react to. The warning structure is kept
     *  to a minimum purposefully and doesn't replace a full comprehension of
     *  the surrounding environment.
     */
    struct Warnings
    {
        bool anyWarningRaised;

        bool carCrossingPlannedPath; ///< Flag a car which has started crossing our planned path
        int carCrossingPlannedPathId;

        bool slowCarAhead; ///< Flag a car detected in my lane and drives slower than me
        int slowCarAheadId;
        double slowCarAheadSpeed;

        /** Default constructor
         *
         */
        Warnings()
        : anyWarningRaised(false)
        , carCrossingPlannedPath(false)
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

    /** Checks myAV surrounding and triggers warnings if necessary
     *
     * @param[out] warnings Warnings raised if a car is crossing my path of if a slow moving car is ahead in my lane
     *
     */
    void environmentalEvaluation(Warnings& warnings) const;

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

    SensorFusion& mSensorFusion; ///< Instance of the Prediction sub-system
    std::vector<DetectedVehicleData> mNearbyCars; /// Cars that are worth being taken into consideration when it comes to planning the trajectory

    constexpr static const double mMaximumAccelerationMs = policy::getSafePolicy(policy::maxAccelerationMs); ///< Maximum allowed acceleration in m/s^2 @note 10
    constexpr static const double mMaximumDetectionDistance = 80.0; ///< Distance below which I start to consider cars as being close and consider them into the Bahavior Planning task
};

#endif //PREDICTOR_H
