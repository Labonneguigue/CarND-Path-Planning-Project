
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

void PathPlanner::solvePath(ControllerFeedback controllerFeedback,
                            std::vector<double>& next_x,
                            std::vector<double>& next_y){

    // I update to Predictor to build an up-to-date representation of the surroundings
    // I check if there are any warning flags raised, if so I run the Behavior Planner module
    Predictor::Warnings warnings;
    mPredictor.environmentalEvaluation(warnings);

    if(warnings.anyWarningRaised)
    {
#if VERBOSE > 1
        std::cout << "Warnings ! I run my Behavior Planner ...\n";
#endif
        mResult = mBehaviorPlanner.computeNewTrajectory(warnings);
        mCounter = 0;
    }
    else
    {
        // If no warnings are raised, I run the behavior only once in a while
        --mCounter;
        if (mCounter <= 0)
        {
            mResult = mBehaviorPlanner.computeNewTrajectory(warnings);
            mCounter = 20;
        }
    }

    // Using high level planning from the Behavior Planning module, the remaining planned path
    // and the mapping data, I plan the new trajectory
    mTrajectoryGenerator.computeTrajectory(controllerFeedback, mResult, next_x, next_y);

}
