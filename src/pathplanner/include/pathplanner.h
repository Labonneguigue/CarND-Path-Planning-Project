
#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>

#include "mapdata.h" // MapData
#include "controllerfeedback.h" // ControllerFeedback
#include "behaviorplanner.h" // BehaviorPlanner
#include "predictor.h" // Predictor
#include "trajectorygenerator.h" // TrajectoryGenerator


class PathPlanner
{
public:

    /** Constructor receiving references to the BehaviorPlanner
     *  and TrajectoryGenerator sub-modules instance to be used
     *   as part of the PathPlanner to fulfill its tasks.
     *
     */
    PathPlanner(BehaviorPlanner& behaviorPlanner,
                Predictor& predictor,
                TrajectoryGenerator& trajectoryGenerator);

    /** Default destructor
     *
     */
    ~PathPlanner();


    /** Solve the Path Planning task of my Self Driving Car
     *
     * @param[in] mapData Most recent map data
     * @param[in] controllerFeedback Remaining trajectory information
     * @param[out] next_x Reference to a vector of the x coordinates of the planned path
     * @param[out] next_y Reference to a vector of the y coordinates of the planned path
     *
     * @note The planned path coordinates contained in the vectors next_x and next_y will
     *       be given to the simulator to actuate the car. They are given in world
     *       reference frame.
     */
    void solvePath(ControllerFeedback controllerFeedback,
                   std::vector<double>& next_x,
                   std::vector<double>& next_y);

private:

    BehaviorPlanner& mBehaviorPlanner; ///< BehaviorPlanner instance allowing construction of optimal trajectory
    Predictor& mPredictor;
    TrajectoryGenerator& mTrajectoryGenerator; ///< TrajectoryGenerator computes optimal trajectory waypoints

    BehaviorPlanner::HighLevelTrajectoryReport mResult;

    int mCounter; ///< Counter used for scheduling
};

#endif //PATH_PLANNER_H
