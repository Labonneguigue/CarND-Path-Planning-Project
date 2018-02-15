#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "vehicledata.h" // VehicleData
#include "mapdata.h" // MapData
#include "controllerfeedback.h" // ControllerFeedback
#include "behaviorplanner.h" // BehaviorPlanner
#include "sensorfusion.h" // SensorFusion
#include "trajectorygenerator.h" // TrajectoryGenerator
#include "utl.h"

class PathPlanner
{
public:

    /** Constructor receiving references to instances of the SensorFusion,
     *  BehaviorPlanner, and TrajectoryGenerator classes to be used as part
     *  of the PathPlanner to fulfill its tasks.
     *
     */
    PathPlanner(SensorFusion& sensorFusion,
                BehaviorPlanner& behaviorPlanner,
                TrajectoryGenerator& trajectoryGenerator);

    ~PathPlanner();


    void solvePath(MapData mapData,
                   ControllerFeedback controllerFeedback,
                   std::vector<double>& next_x,
                   std::vector<double>& next_y);

private:

    SensorFusion& mSensorFusion; ///< SensorFusion instance keeping updates records of the surrounding cars
    BehaviorPlanner& mBehaviorPlanner; ///< BehaviorPlanner instance allowing construction of optimal trajectory
    TrajectoryGenerator& mTrajectoryGenerator; ///< TrajectoryGenerator computes optimal trajectory waypoints

    static constexpr double mCenterLane = 6; ///< Center Lane expressed in Frenet coordinates for d
    static constexpr double mNumbersOfWaypoints = 50; ///< Number of waypoints in the result path
    static constexpr double mMaximumVelocityMs = policy::getSafePolicy(utl::mph2ms(policy::maxSpeedMph)); ///< Target velocity in mph @note 50 is the speed limit
    double mMaximumAccelerationMs = policy::getSafePolicy(policy::maxAccelerationMs); ///< Maximum allowed acceleration in m/s^2 @note 10 m/s is the required max
    double mCurrentTargetVelocityMs; ///< Ongoing velocity target in [m/s], influenced by induced behavior
    double mMyAVSpeedAtEndOfPlannedPathMs; ///< Speed of my car at the end of the planned path in m/s.
};

#endif //PATH_PLANNER_H
