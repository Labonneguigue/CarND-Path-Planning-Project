#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "vehicledata.h" // VehicleData
#include "mapdata.h" // MapData
#include "controllerfeedback.h" // ControllerFeedback

class PathPlanner
{
public:

    PathPlanner();

    ~PathPlanner();

    void solverPath(VehicleData vehicleData,
                    MapData mapData,
                    ControllerFeedback controllerFeedback,
                    std::vector<double>& next_x,
                    std::vector<double>& next_y);

private:

    static constexpr double mCenterLane = 6; ///< Center Lane expressed in Frenet coordinates for d
    static constexpr double mNumbersOfWaypoints = 50; ///< Number of waypoints in the result path
    static constexpr double mReferenceVelocityMph = 49.5; ///< Target velocity in mph @note 50 is the speed limit
    
};

#endif //PATH_PLANNER_H
