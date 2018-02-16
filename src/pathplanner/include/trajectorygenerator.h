
#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include "controllerfeedback.h"
#include "mapdata.h"
#include "sensorfusion.h"
#include "drivingpolicy.h"
#include "behaviorplanner.h"

class TrajectoryGenerator
{
public:

    TrajectoryGenerator(SensorFusion& sensorFusion);

    ~TrajectoryGenerator();

    void initialiseTrajectoryWithRemainingOne(ControllerFeedback& controllerFeedback);

    void computeTrajectory(BehaviorPlanner::HighLevelTrajectoryReport& result,
                           MapData& mapData,
                           std::vector<double>& next_x,
                           std::vector<double>& next_y);

    /** In order to avoid |accelerating| too much, the speed of the car must
     * be carefully and incrementally changed
     *
     * @note This method changes the class member mMyAVSpeedAtEndOfPlannedPathMs
     */
    void computeStepSpeed();

    void setCurrentTargetVelocity(const double velocityMs)
    { mCurrentTargetVelocityMs = velocityMs; }

private:

    SensorFusion& mSensorFusion;

    // Vector of widely spaced waypoints to be interpolated with a spline
    // to smooth out the car trajectory
    std::vector<double> mMajorWayPoints_x;
    std::vector<double> mMajorWayPoints_y;

    std::vector<double> mPlannedTrajectory_x; ///< Generated Trajectory x axis
    std::vector<double> mPlannedTrajectory_y; ///< Generated Trajectory y axis

    static constexpr double mNumbersOfWaypoints = 50; ///< Number of waypoints in the result path

    // I need to extend the planned path. The following car x, y, and yaw coordinates
    // are usefull to that regard. They'll be different depending on whether the
    // remaining planned path not yet executed by the controller is small or not.
    double endPathCar_x;
    double endPathCar_y;
    double endPathCar_yaw;
    double mMyAVSpeedAtEndOfPlannedPathMs; ///< Speed of my car at the end of the planned path in m/s.
    double remainingPathSize; ///< The difference with mNumberOfWaypoints gives the number of waypoints to compute and append to the trajectory
    static constexpr double mMaximumVelocityMs = policy::getSafePolicy(utl::mph2ms(policy::maxSpeedMph)); ///< Target velocity in mph @note 50 is the speed limit
    static constexpr double mMaximumAccelerationMs = policy::getSafePolicy(policy::maxAccelerationMs); ///< Maximum allowed acceleration in m/s^2 @note 10 m/s is the required max
    double mCurrentTargetVelocityMs; ///< Ongoing velocity target in [m/s], influenced by induced behavior
    static constexpr const double mSimulatorWaypointsDeltaT = policy::simulatorDeltaT;
    static constexpr const double mMaximumSpeedIncrement = mSimulatorWaypointsDeltaT * mMaximumAccelerationMs;
};

#endif //TRAJECTORY_GENERATOR_H
