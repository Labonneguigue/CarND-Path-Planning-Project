
#include <cmath>
#include <limits>
#include <iostream>
#include "behaviorplanner.h"

#define DEBUG 1

BehaviorPlanner::BehaviorPlanner(Predictor& predictor, SensorFusion& sensorFusion)
: mPredictor(predictor)
, mSensorFusion(sensorFusion)
, mWarnings()
, mResults(std::vector<HighLevelTrajectoryReport>(mSensorFusion.highway().getNumberLanes()))
, mResultIndex(static_cast<int>(mSensorFusion.highway().initialLane))
{
    // Initialisation of each HighLevelTrajectoryReports to have the correct targetLane
    for (int report = 0; report < mResults.size() ; ++report)
    {
        mResults[report].targetLane = static_cast<Lane>(report);
    }
}

BehaviorPlanner::~BehaviorPlanner()
{}


const BehaviorPlanner::HighLevelTrajectoryReport BehaviorPlanner::computeNewTrajectory(Predictor::Warnings warnings)
{
    // Set the warnings of each report to the be reflecting what the predictor found
    setWarnings(warnings.anyWarningRaised);

    // Then, find best road behavior
    const Lane currentLane = mSensorFusion.myAV().lane;
    const int currentSpeedMs = mSensorFusion.myAV().speedMs;
    std::vector<Lane> lanes = mSensorFusion.highway().getAvailableLanes();
    std::vector<double> costs(lanes.size());
    double minimumCostIndex = 0;
    double minimumCost = std::numeric_limits<double>::max();

#if DEBUG
    std::cout << "Behavior Planning on lane " << currentLane << " with s : " << mSensorFusion.myAV().s << "\n";
#endif

    // Get the Prediction module prepared to compute the cost functions
    mPredictor.prepareSensorDataForPrediction();

    Lane preferedTargetLane;

    if (policy::keepRightLane)
    {
        preferedTargetLane = lanes[lanes.size()-1];
    }
    else
    {
        preferedTargetLane = currentLane;
    }

    for (int i = 0; i < lanes.size() ; ++i)
    {
        if (abs(currentLane - lanes[i]) >= 2)
        {
            costs[i] = 1;
        }
        else
        {
            costs[i] = cost(currentLane,
                            lanes[i],
                            preferedTargetLane,
                            currentSpeedMs,
                            mWarnings,
                            mResults[i]);
        }

        if (costs[i] < minimumCost)
        {
            minimumCost = costs[i];
            minimumCostIndex = i;
        }
    }

    std::cout << "\n";

    return mResults[minimumCostIndex];
}

double BehaviorPlanner::cost(const Lane currentLane,
                             const Lane targetLane,
                             const Lane preferedTargetLane,
                             const double currentSpeedMs,
                             const Predictor::Warnings& warnings,
                             HighLevelTrajectoryReport& report) const
{
    assert(currentLane >= 0);
    assert(targetLane >= 0);

    std::cout << "Cost calculation for lane : " << targetLane << "\n";

    constexpr const double targetS = 400; // The cost function always aims at this far target

    // * If the car drives in the US, there are no specific lane goal.
    //   The prefered target lane is the current one in order to reduce
    //   the number of lane change. Switching lane is (slightly) penalised.
    // * If the car drives in EU for example, the should stay on the right
    //   most lane if the car is not undertaking. In that case the prefered
    //   target lane is the right most one. Not being on that lane will be
    //   slightly penalized.
    const double delta_d = (std::abs(preferedTargetLane - targetLane) * 0.1) + 1.0;
    double cost;
    double laneSpeedMs;
    const double minimumCostTime = targetS / utl::mph2ms(policy::maxSpeedMph);
    double totalTimeToReachTargetS;

    // Target is maximum speed, might be reduced if cars are observed
    report.targetSpeedMs = policy::getSafePolicy(policy::maxSpeedMs);

    // 2 different approaches, one if the evaluated lane is my current lane,
    // the other is if it is not.
    if (currentLane == targetLane)
    {
        report.behavior = keepLane;
        double distanceCarAhead;
        mSensorFusion.getDistanceAndSpeedCarAhead(distanceCarAhead, laneSpeedMs);
        // If the lane evaluated is my current one, I only need to calculate the time to
        // get to the target s at the speed of the car in front
        // I take into account that I might have to slow down to reach the lane speed
        laneSpeedMs = std::min<double>(laneSpeedMs, policy::getSafePolicy(policy::maxSpeedMs));
        totalTimeToReachTargetS = targetS / laneSpeedMs;
        report.targetSpeedMs = laneSpeedMs;
    }
    else
    {
        /* Cost should be calculated as the time to target s:
         - the time to insertion
         - the ramp up/down time from current speed to lane speed
         - the current speed of the lane
         */

        mPredictor.getLaneSpeedAndTimeToInsertion(targetLane
                                                 ,laneSpeedMs
                                                 ,report.timeToInsertion);

        // If the car want to change 2 lanes, it needs to take into
        // consideration the time to change to the first lane
        if (abs(currentLane-targetLane) >= 2)
        {
            ///@todo Come up with a solution here
            report.timeToInsertion += 0.0;
        }

        constexpr const double thresholdForImmediateLaneChange = 0.1;

        if (targetLane > currentLane)
        {
            if (report.timeToInsertion < thresholdForImmediateLaneChange)
            {
                report.behavior = rightLaneChange;
            }
            else
            {
                report.behavior = prepareRightLaneChange;
            }
        }
        else if(targetLane < currentLane)
        {
            if (report.timeToInsertion < thresholdForImmediateLaneChange)
            {
                report.behavior = leftLaneChange;
            }
            else
            {
                report.behavior = prepareLeftLaneChange;
            }
        }

        // Distance travelled while waiting for insertion, 0 if timeToInsertion is 0
        const double sameLaneDistance = laneSpeedMs * report.timeToInsertion;

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

        std::cout << "ramp distance " << rampDistance << "\n";

        // v = d / t or t = d / v
        double timeToReachTargetSAtFullSpeed = ((targetS - sameLaneDistance) - rampDistance) / laneSpeedMs;
        //std::cout << "timeToReachTargetSAtFullSpeed : " <<  timeToReachTargetSAtFullSpeed << "\n";
        // Penalty for changing lane, prevents over changing
        timeToReachTargetSAtFullSpeed *= delta_d;
        
        totalTimeToReachTargetS = report.timeToInsertion + timeToReachLaneSpeed + timeToReachTargetSAtFullSpeed;
    }

    std::cout << "totalTimeToReachTargetS : " <<  totalTimeToReachTargetS << "\n";
    assert(totalTimeToReachTargetS > minimumCostTime);

    cost = utl::sigmoid(totalTimeToReachTargetS / minimumCostTime);

#if DEBUG
    std::cout << "\t\tCost of lane : " << targetLane << " is : " << cost << "\n";
#endif
    
    return cost;
}
