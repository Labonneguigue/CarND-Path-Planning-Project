
#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
#include "predictor.h"
#include "vehicledata.h"
#include "sensorfusion.h"
#include "drivingpolicy.h"

class BehaviorPlanner
{
public:

    enum Behavior
    {
        keepLane,
        leftLaneChange,
        rightLaneChange,
        prepareLeftLaneChange,
        prepareRightLaneChange
    };

    /** Results of the Behavior Planning sub-module to be given to
     *  the Trajectory Generation sub-module for precise path
     *  definition
     *
     */
    struct HighLevelTrajectoryReport
    {
        Behavior behavior; ///< Type of behavior to adopt
        double targetSpeedMs; ///< Optimal speed to reach target
        Lane targetLane; ///< Goal lane for each reports
        double timeToInsertion; ///< Estimation of the time it'll take before possible lane change
        double recommendedTargetSpeed; ///< Speed to adopt for optimal & quick lane change
        bool warningTriggered; ///< Whether or not the Behavior Planner has been triggered by a warning

        HighLevelTrajectoryReport()
        : behavior(keepLane)
        , targetSpeedMs(policy::getSafePolicy(policy::maxSpeedMs))
        , targetLane(secondLane)
        , timeToInsertion(0.0)
        , recommendedTargetSpeed(0.0)
        , warningTriggered(false)
        {}
    };

    /** Default constructor
     *
     */
    BehaviorPlanner(Predictor& predictor,
                    SensorFusion& sensorFusion);

    /** Default destructor
     *
     */
    ~BehaviorPlanner();

    /** Update the state of the vehicle using the SensorFusion data
     *  being processed by the Predictor.
     *
     * @return Trajectory decision
     *
     */
    const BehaviorPlanner::HighLevelTrajectoryReport computeNewTrajectory(Predictor::Warnings warnings = Predictor::Warnings());

    /** Function evaluates the cost to choose each possible trajectory
     *
     * @param[in] currentLane The current lane myAV is on
     * @param[in] currentSpeedMs The current myAV speed
     * @param[in] targetLane The target lane for which the cost is evaluated
     * @param[in] timeToInsertion The time to wait before changing lane
     *
     * @return double Cost to choose this trajectory
     * 
     * @note The cost is bounded between 0 and 1. The lower the cost, the better
     *       the trajectory. The time to insertion differentiate between a laneChange
     *       and a prepareLaneChange type of behavior.
     */
    void cost(const Lane currentLane,
              const Lane targetLane,
              const Lane preferedTargetLane,
              const double currentSpeedMs,
              const Predictor::Warnings& warnings,
              std::vector<Lane>& lanes,
              std::vector<HighLevelTrajectoryReport>& reports,
              std::vector<double>& costs);

    /**
     *
     */
    inline void setWarnings(bool warningLevel)
    {
        for (unsigned int report = 0; report < mResults.size(); ++report)
        {
            mResults[report].warningTriggered = warningLevel;
        }
    }

private:
    
    Predictor& mPredictor; ///< Instance of the Prediction sub-system
    SensorFusion& mSensorFusion; ///< Instance of the SensorFusion database

    Predictor::Warnings mWarnings; ///< Current warnings raised
    

    std::vector<HighLevelTrajectoryReport> mResults; ///< Results to be given to the Trajectory Generation sub-module
    int mResultIndex; ///< Index of the mResults vector that gives the best trajectory
};

#endif //BEHAVIOR_PLANNER_H
