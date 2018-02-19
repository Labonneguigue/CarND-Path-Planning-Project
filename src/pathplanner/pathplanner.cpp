
#include "pathplanner.h" // PathPlanner

PathPlanner::PathPlanner(BehaviorPlanner& behaviorPlanner,
                         Predictor& predictor,
                         TrajectoryGenerator& trajectoryGenerator)
: mBehaviorPlanner(behaviorPlanner)
, mPredictor(predictor)
, mTrajectoryGenerator(trajectoryGenerator)
, mResult()
, mCounter(0)
{}

PathPlanner::~PathPlanner()
{}

void PathPlanner::solvePath(MapData mapData,
                            ControllerFeedback controllerFeedback,
                            std::vector<double>& next_x,
                            std::vector<double>& next_y){

    // I update to Predictor to build an up-to-date representation of the surroundings
    mPredictor.prepareSensorDataForPrediction();

    // I check if there are any warning flags raised, if so I run the Behavior Planner module
    Predictor::Warnings warnings;
    if(warnings.anyWarningRaised)
    {
        std::cout << "Warnings ! I run my Behavior Planner ...\n";
        mResult = mBehaviorPlanner.computeNewTrajectory(warnings);
        mCounter = 0;
    }
    else
    {
        // If no warnings are raised, I run the behavior only once in a while
        ++mCounter;
        if (mCounter >= 20)
        {
            mResult = mBehaviorPlanner.computeNewTrajectory(warnings);
            mCounter = 0;
        }
    }

    // I initialise the path to the remaining one
    mTrajectoryGenerator.initialiseTrajectoryWithRemainingOne(controllerFeedback);

    // Using high level planning for the Behavior Planning module, I plan the new trajectory
    mTrajectoryGenerator.computeTrajectory(mResult, mapData, next_x, next_y);

}
