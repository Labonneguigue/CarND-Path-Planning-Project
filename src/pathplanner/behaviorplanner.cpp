#include <limits>
#include <iostream>
#include <vector>
#include "behaviorplanner.h"

#define DEBUG

BehaviorPlanner::BehaviorPlanner(Predictor& predictor, SensorFusion& sensorFusion)
: mPredictor(predictor)
, mSensorFusion(sensorFusion)
{}

BehaviorPlanner::~BehaviorPlanner()
{}

void BehaviorPlanner::updateState()
{
    Highway highway = mSensorFusion.highway();
    const int currentLane = mSensorFusion.myAV().lane;
    std::vector<int> lanes = highway.getAvailableLanes(currentLane);
    std::vector<double> costs(lanes.size());

    double minimumCostIndex = 0;
    double minimumCost = std::numeric_limits<double>::max();

#ifdef DEBUG
    std::cout << "Costs : ";
#endif
    for (int i = 0; i < lanes.size() ; ++i)
    {
        costs[i] = cost(currentLane, currentLane+lanes[i]);
        if (costs[i] < minimumCost)
        {
            minimumCost = costs[i];
            minimumCostIndex = i;
        }
#ifdef DEBUG
        std::cout << costs[i] << "  , ";
#endif
    }

#ifdef DEBUG
    std::cout << "\n";
#endif

    // New target lane is minimumCostIndex;
    mSensorFusion.myAV().lane = static_cast<Lane>(minimumCostIndex);

}

double BehaviorPlanner::cost(const double currentLane, const double targetLane) const
{
    assert(currentLane >= 0);
    assert(targetLane >= 0);

    const double targetS = 100; // The cost function always aims at this far target
    // Since there are no specific lane goal, I set it to be the current one so that
    // switching lane is (slightly) penalised.
    const int targetD = currentLane;
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

    ///@todo remove
    cost = laneSpeed * timeToInsertion;

    return cost;
}
