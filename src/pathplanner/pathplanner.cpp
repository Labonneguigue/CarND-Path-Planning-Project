
#include <iostream>
#include "pathplanner.h" // PathPlanner

PathPlanner::PathPlanner(SensorFusion& sensorFusion,
                         BehaviorPlanner& behaviorPlanner,
                         TrajectoryGenerator& trajectoryGenerator)
: mSensorFusion(sensorFusion)
, mBehaviorPlanner(behaviorPlanner)
, mTrajectoryGenerator(trajectoryGenerator)
{}

PathPlanner::~PathPlanner()
{}

void PathPlanner::solvePath(MapData mapData,
                            ControllerFeedback controllerFeedback,
                            std::vector<double>& next_x,
                            std::vector<double>& next_y){

    
    mTrajectoryGenerator.initialiseTrajectoryWithRemainingOne(controllerFeedback);

    BehaviorPlanner::HighLevelTrajectoryReport result = mBehaviorPlanner.updateState();

    mTrajectoryGenerator.computeTrajectory(result, mapData, next_x, next_y);
 
}
