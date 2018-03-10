
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

    /** Compute the trajectory that the car should follow. The high level decisions
     *  for this trajectory have been set by the Behavior Planning sub-module.
     *  This method is responsible for generating a trajectory that is smooth,
     *  doesn't exceed the maximum allowed jerk and acceleration and sensible
     *  in an environment with other cars on the road.
     *
     * @param[in] result High level decision regarding the trajectory (speed,target lane, ..)
     * @param[in] mapData Mapping data for self-localization on the highway
     * @param[out] next_x Reference to a vector of the x coordinates of the planned path
     * @param[out] next_y Reference to a vector of the y coordinates of the planned path
     *
     */
    void computeTrajectory(const ControllerFeedback& controllerFeedback,
                           const BehaviorPlanner::HighLevelTrajectoryReport& result,
                           const MapData& mapData,
                           std::vector<double>& next_x,
                           std::vector<double>& next_y);

private:
    
    /** Initialise the trajectory using the remaining one (previously set by the
     *  computeTrajectory() method) for reuse.
     *
     * @param[in] controllerFeedback Remaining trajectory as vector of x and y coordinates
     *
     */
    void initialiseTrajectoryWithRemainingOne(const ControllerFeedback& controllerFeedback);


    /** In order to avoid |accelerating| too much, the speed of the car must
     * be carefully and incrementally changed
     *
     * @note This method changes the class member mMyAVSpeedAtEndOfPlannedPathMs
     */
    void computeStepSpeed();

    /** Set the speed that my car should meet as soon as possible
     *
     * @note Ramping up and down my speed will be limited by the maximum
     *       authorized acceleration and jerk.
     *
     * @param[in] velocityMs Target speed in m/s
     *
     */
    inline void setCurrentTargetVelocity(const double velocityMs)
    { mCurrentTargetVelocityMs = velocityMs; }

    SensorFusion& mSensorFusion; ///< Reference to the sensor fusion database

    // Vector of widely spaced waypoints to be interpolated with a spline
    // to smooth out the car trajectory
    std::vector<double> mMajorWayPoints_x;
    std::vector<double> mMajorWayPoints_y;

    std::vector<double> mPlannedTrajectory_x; ///< Generated Trajectory x axis
    std::vector<double> mPlannedTrajectory_y; ///< Generated Trajectory y axis

    // I need to extend the planned path. The following car x, y, and yaw coordinates
    // are usefull to that regard. They'll be different depending on whether the
    // remaining planned path not yet executed by the controller is small or not.
    VehicleData mEndPathCar;

    double remainingPathSize; ///< The difference with mNumberOfWaypoints gives the number of waypoints to compute and append to the trajectory
    double mCurrentTargetVelocityMs; ///< Ongoing velocity target in [m/s], influenced by induced behavior
    Lane mCurrentTargetLane;
    
    static constexpr double mMaximumAccelerationMs = policy::getSafePolicy(policy::maxAccelerationMs); ///< Maximum allowed acceleration in m/s^2 @note 10 m/s is the required max
    static constexpr const double mSimulatorWaypointsDeltaT = policy::simulatorDeltaT;
    static constexpr const double mMaximumSpeedIncrement = mSimulatorWaypointsDeltaT * mMaximumAccelerationMs;
    static constexpr double mMaximumVelocityMs = policy::getSafePolicy(utl::mph2ms(policy::maxSpeedMph)); ///< Target velocity in mph @note 50 is the speed limit
    static constexpr double mNumbersOfWaypoints = 50; ///< Number of waypoints in the result path

};

#endif //TRAJECTORY_GENERATOR_H
