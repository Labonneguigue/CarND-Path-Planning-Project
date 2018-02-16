#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

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

    struct HighLevelTrajectoryReport
    {
        Behavior behavior;
        double targetSpeedMs;
        Lane targetLane;

        HighLevelTrajectoryReport()
        : behavior(keepLane)
        , targetSpeedMs(policy::getSafePolicy(policy::maxSpeedMs))
        , targetLane(secondLane)
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
     */
    const BehaviorPlanner::HighLevelTrajectoryReport& updateState();

    /**
     *
     */
    void computeNewTrajectory(bool warnings);

    /** Function evaluates the cost to choose each possible trajectory
     *
     * @param[in] currentLane The current lane myAV is on
     * @param[in] currentSpeed The current myAV speed
     * @param[in] targetLane The target lane for which the cost is evaluated
     *
     * return double Cost to choose this trajectory
     * 
     * @note The cost is bounded between 0 and 1. The lower the cost, the better
     *       the trajectory
     */
    double cost(const Lane currentLane, const double currentSpeed, const double targetLane) const;

private:
    
    Predictor& mPredictor; ///< Instance of the Prediction sub-system
    SensorFusion& mSensorFusion; ///< Instance of the SensorFusion database

    Predictor::Warnings mWarnings; ///< Current warnings raised
    int mCounter; ///< Counter used for scheduling

    HighLevelTrajectoryReport mResults; ///< Results to be given to the Trajectory Generation sub-module
};

#endif //BEHAVIOR_PLANNER_H
