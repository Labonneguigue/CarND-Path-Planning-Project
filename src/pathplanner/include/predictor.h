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
    void environmentalEvaluation(Warnings& warnings);

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
     * @param[out] recommendedSpeedMs Speed to target to achieve insertion
     *
     */
    void getLaneSpeedAndTimeToInsertion(const int lane,
                                        double& laneSpeedMs,
                                        double& timeToInsertionS,
                                        double& recommendedSpeedMs) const;

    /** Returns true if my car can change to specified lane given at certain position
     *
     * @param[in] targetLane Lane targeted for the lane change
     * @param[in] positionS  Current position of the vehicle (S frenet)
     *
     * @return True if changing lane immediatelly is safe, False otherwise
     */
    bool canIChangeLane(const Lane targetLane, const double positionS);

    /**
     *
     */
    inline static bool isThisCarBlockingMe(const DetectedVehicleData car,
                                                 const double positionS)
    {
        return ( (car.s < (positionS + policy::safeDistance))
              && (car.s > (positionS - policy::safeDistance)) ) ? true : false;
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
    bool getDistanceAndSpeedCarAhead(double& distance,
                                                double& speed);

private:

    SensorFusion& mSensorFusion; ///< Instance of the Prediction sub-system
    VehicleData& mMyAV; ///< Reference to the representation of myAV in SensorFusion
    std::vector<std::vector<DetectedVehicleData>> mCarsByLane; /// Cars that are worth being taken into consideration when it comes to planning the trajectory sorted by lanes

    constexpr static const double mMaximumAccelerationMs = policy::getSafePolicy(policy::maxAccelerationMs); ///< Maximum allowed acceleration in m/s^2 @note 10
};

#endif //PREDICTOR_H
