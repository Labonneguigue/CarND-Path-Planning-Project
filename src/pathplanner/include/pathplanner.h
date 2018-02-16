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

};

#endif //PATH_PLANNER_H
