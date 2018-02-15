#include <limits>
#include <iostream>
#include <vector>
#include "behaviorplanner.h"

#define DEBUG 1

BehaviorPlanner::BehaviorPlanner(Predictor& predictor, SensorFusion& sensorFusion)
: mPredictor(predictor)
, mSensorFusion(sensorFusion)
{}

BehaviorPlanner::~BehaviorPlanner()
{}

void BehaviorPlanner::updateState()
{
    // First check if there are some warnings from the Prediction module
    Predictor::Warnings warnings;
    if (mPredictor.anyWarnings(warnings))
    {
        std::cout << "Warnings ! I run my Behavior Planner ...\n";
        mCounter = 20;
    }

    ++mCounter;
    if (mCounter >= 20)
    {
        computeNewTrajectory();
        mCounter = 0;
    }
}

void BehaviorPlanner::computeNewTrajectory()
{
    // Then, find best road bahavior
    Highway highway = mSensorFusion.highway();
    const Lane currentLane = mSensorFusion.myAV().lane;
    const int currentSpeed = mSensorFusion.myAV().speed;
    std::vector<int> lanes = highway.getAvailableLanes(currentLane);
    std::vector<double> costs(lanes.size());

    // Get the Prediction module prepared to compute the cost functions
    mPredictor.prepareSensorDataForPrediction();

    double minimumCostIndex = 0;
    double minimumCost = std::numeric_limits<double>::max();

#if DEBUG
    std::cout << "Behavior Planning on lane " << currentLane << "\n";
#endif

    for (int i = 0; i < lanes.size() ; ++i)
    {
        costs[i] = cost(currentLane, currentSpeed, lanes[i]);
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
                             const double currentSpeed,
                             const double targetLane) const
{
    assert(currentLane >= 0);
    assert(targetLane >= 0);

    const double targetS = 200; // The cost function always aims at this far target
    // Since there are no specific lane goal, I set it to be the current one so that
    // switching lane is (slightly) penalised.
    const int targetD = currentLane;
    const double delta_d = (abs(targetD - targetLane) * 0.05) + 1.0;
    double cost;

    double laneSpeed;
    double timeToInsertion;

    mPredictor.getLaneSpeedAndTimeToInsertion(targetLane, laneSpeed, timeToInsertion);

    /* Cost should be calculated as the time to target s:
        - the time to insertion
        - the ramp up/down time from current speed to lane speed
        - the current speed of the lane
     */

    // Idea: 0 cost would be acceleration to max speed and keep it until target.

    // Distance travelled while waiting for insertion, 0 if timeToInsertion is 0
    const double sameLaneDistance = laneSpeed * timeToInsertion;

    // Sanity check. If I have to wait for too long, this is not a good choice
    if (sameLaneDistance >= 150)
    {
        return 1;
    }

    // Distance travelled while ramping up or down the speed to lane speed
    constexpr static const double maximumSafeAcceleration = policy::getSafePolicy(policy::maxAccelerationMs);
    const double timeToReachLaneSpeed = (laneSpeed - currentSpeed) / maximumSafeAcceleration;
    // s = v * t + 0.5 * a * t * t
    const double rampDistance = (currentSpeed * timeToReachLaneSpeed) +
                                (0.5 * maximumSafeAcceleration * utl::sqr(timeToReachLaneSpeed));

    // v = d / t or t = d / v
    double timeToReachTargetS = ((targetS - sameLaneDistance) - rampDistance) / laneSpeed;
    timeToReachTargetS *= delta_d;

    const double minimumCostTime = targetS / utl::mph2ms(policy::maxSpeedMph);

    assert(timeToReachTargetS > minimumCostTime);

    cost = utl::sigmoid(timeToReachTargetS / minimumCostTime);

#if DEBUG
    std::cout << "Cost of lane : " << targetLane << " is : " << cost << "\n";
#endif

    return cost;
}
