#include <cmath>
#include <limits>
#include <iostream>
#include <vector>
#include "behaviorplanner.h"

#define DEBUG 1

BehaviorPlanner::BehaviorPlanner(Predictor& predictor, SensorFusion& sensorFusion)
: mPredictor(predictor)
, mSensorFusion(sensorFusion)
, mWarnings()
, mCounter(0)
, mResults()
{}

BehaviorPlanner::~BehaviorPlanner()
{}

const BehaviorPlanner::HighLevelTrajectoryReport& BehaviorPlanner::updateState()
{
    // First check if there are some warnings from the Prediction module
    if (mPredictor.anyWarnings(mWarnings))
    {
        std::cout << "Warnings ! I run my Behavior Planner ...\n";
        computeNewTrajectory(true);
        mCounter = 0;
    }
    else
    {
        ++mCounter;
        if (mCounter >= 20)
        {
            computeNewTrajectory(false);
            mCounter = 0;
        }
    }
    return mResults;
}

void BehaviorPlanner::computeNewTrajectory(bool warnings)
{
    HighLevelTrajectoryReport result;
    if (warnings)
    {
        result.targetSpeedMs = mWarnings.slowCarAheadSpeed;
    }

    // Then, find best road behavior
    Highway highway = mSensorFusion.highway();
    const Lane currentLane = mSensorFusion.myAV().lane;
    const int currentSpeedMs = mSensorFusion.myAV().speedMs;

    std::vector<int> lanes = highway.getAvailableLanes(currentLane);
    assert(lanes.size() > 0);

    std::vector<double> costs(lanes.size());

    // Get the Prediction module prepared to compute the cost functions
    mPredictor.prepareSensorDataForPrediction();

    double minimumCostIndex = 0;
    double minimumCost = std::numeric_limits<double>::max();

#if DEBUG
    std::cout << "Behavior Planning on lane " << currentLane << " with s : " << mSensorFusion.myAV().s << "\n";
#endif

    for (int i = 0; i < lanes.size() ; ++i)
    {
        costs[i] = cost(currentLane, currentSpeedMs, lanes[i]);
        if (costs[i] < minimumCost)
        {
            minimumCost = costs[i];
            minimumCostIndex = i;
        }
    }

#if DEBUG
    std::cout << "\n";
#endif

    // New target lane is minimumCostIndex;
    mSensorFusion.myAV().lane = static_cast<Lane>(minimumCostIndex);
}

double BehaviorPlanner::cost(const Lane currentLane,
                             const double currentSpeedMs,
                             const double targetLane) const
{
    assert(currentLane >= 0);
    assert(targetLane >= 0);

    const double targetS = 200; // The cost function always aims at this far target
    // Since there are no specific lane goal, I set it to be the current one so that
    // switching lane is (slightly) penalised.
    const int targetD = currentLane;
    const double delta_d = (std::abs(targetD - targetLane) * 0.05) + 1.0;
    double cost;

    double laneSpeedMs;
    double timeToInsertion;

    mPredictor.getLaneSpeedAndTimeToInsertion(targetLane, laneSpeedMs, timeToInsertion);

    /* Cost should be calculated as the time to target s:
     - the time to insertion
     - the ramp up/down time from current speed to lane speed
     - the current speed of the lane
     */

    // Idea: 0 cost would be acceleration to max speed and keep it until target.

    // Distance travelled while waiting for insertion, 0 if timeToInsertion is 0
    const double sameLaneDistance = laneSpeedMs * timeToInsertion;

    // Sanity check. If I have to wait for too long, this is not a good choice
    if (sameLaneDistance >= 150)
    {
        return 1;
    }

    // Distance travelled while ramping up or down the speed to lane speed
    constexpr static const double maximumSafeAcceleration = policy::getSafePolicy(policy::maxAccelerationMs);
    const double timeToReachLaneSpeed = (laneSpeedMs - currentSpeedMs) / maximumSafeAcceleration;
    // s = v * t + 0.5 * a * t * t
    const double rampDistance = (currentSpeedMs * timeToReachLaneSpeed) +
    (0.5 * maximumSafeAcceleration * utl::sqr(timeToReachLaneSpeed));

    // v = d / t or t = d / v
    double timeToReachTargetSAtFullSpeed = ((targetS - sameLaneDistance) - rampDistance) / laneSpeedMs;
    timeToReachTargetSAtFullSpeed *= delta_d;

    const double minimumCostTime = targetS / utl::mph2ms(policy::maxSpeedMph);
    const double totalTimeToReachTargetS = timeToInsertion + timeToReachLaneSpeed + timeToReachTargetSAtFullSpeed;
    assert(totalTimeToReachTargetS > minimumCostTime);

    cost = utl::sigmoid(totalTimeToReachTargetS / minimumCostTime);

#if DEBUG
    std::cout << "Cost of lane : " << targetLane << " is : " << cost << "\n";
#endif
    
    return cost;
}
