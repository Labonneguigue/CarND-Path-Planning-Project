
#include <cmath>
#include <limits>
#include <iostream>
#include "behaviorplanner.h"

#define VERBOSE 0

BehaviorPlanner::BehaviorPlanner(Predictor& predictor, SensorFusion& sensorFusion)
: mPredictor(predictor)
, mSensorFusion(sensorFusion)
, mWarnings()
, mResults(std::vector<HighLevelTrajectoryReport>(mSensorFusion.highway().getNumberLanes()))
, mResultIndex(static_cast<int>(mSensorFusion.highway().mInitialLane))
{
    // Initialisation of each HighLevelTrajectoryReports to have the correct targetLane
    for (unsigned int report = 0; report < mResults.size() ; ++report)
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
    std::vector<double> costs(lanes.size(), -1.0);
    double minimumCostIndex = 0;
    double minimumCost = std::numeric_limits<double>::max();

#if VERBOSE > 1
    std::cout << "\n--------------------------------------------------------------------\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << "|| \t BEHAVIOR PLANNING ON LANE \t" << currentLane << "\t with s \t " << mSensorFusion.myAV().s << "\n";
    std::cout << "--------------------------------------------------------------------\n";
#endif

    Lane preferedTargetLane;

    if (policy::keepRightLane)
    {
        preferedTargetLane = lanes[lanes.size()-1];
    }
    else
    {
        preferedTargetLane = currentLane;
    }

    // This function is recursive and will propagate to every lanes.
    // This is so because the time to insertion directly impacts the cost
    // of the the lanes further from it.
    // I start by the current since this is not impacted by any other lane
    // calculations.
    cost(currentLane,
         currentLane,
         preferedTargetLane,
         currentSpeedMs,
         warnings,
         lanes,
         mResults,
         costs);

    for (unsigned int lane = 0 ; lane < costs.size() ; ++lane)
    {
        if (costs[lane] < minimumCost)
        {
            minimumCost = costs[lane];
            minimumCostIndex = lane;
        }
    }

#if VERBOSE > 0
    std::cout << "///////////////////////////////////////////////////////////////////\n";
    std::cout << "///////////////////////////////////////////////////////////////////\n";

#endif

    return mResults[minimumCostIndex];
}

void BehaviorPlanner::cost(const Lane currentLane,
                           const Lane targetLane,
                           const Lane preferedTargetLane,
                           const double currentSpeedMs,
                           const Predictor::Warnings& warnings,
                           std::vector<Lane>& lanes,
                           std::vector<HighLevelTrajectoryReport>& reports,
                           std::vector<double>& costs)
{
    if(currentLane == targetLane)
    {
        // Get the Prediction module prepared to compute the cost functions
        mPredictor.prepareSensorDataForPrediction();
    }

    // Ease the access to arrays refering to the currentLane
    const int currentTargetLaneIndex = static_cast<int>(targetLane);
    HighLevelTrajectoryReport& report = reports[currentTargetLaneIndex];

    if ((targetLane < 0)
        || (targetLane >= mSensorFusion.highway().getNumberLanes())
        || (costs[currentTargetLaneIndex] >= 0.0))
    {
        return;
    }

    assert(currentLane >= 0);
    assert(targetLane >= 0);

#if VERBOSE > 0
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << "|| \tCost calculation for lane : \t" << targetLane << "\n";
    std::cout << "--------------------------------------------------------------------\n";
#endif

    // The cost function always aims at this far target to optimise
    // the trajectory on a long time frame
    constexpr const double targetS = 1000;

    double totalTimeToReachTargetS;

    // Target is maximum speed, might be reduced if cars are observed
    report.targetSpeedMs = policy::getSafePolicy(policy::maxSpeedMs);

    // 2 different approaches, one if the evaluated lane is my current lane,
    // the other is if it is not.
    if (currentLane == targetLane)
    {
        report.behavior = keepLane;
        double distanceCarAhead;
        mPredictor.getDistanceAndSpeedCarAhead(distanceCarAhead, report.targetSpeedMs);
        // If the lane evaluated is my current one, I only need to calculate the time to
        // get to the target s at the speed of the car in front
        // I take into account that I might have to slow down to reach the lane speed
        report.targetSpeedMs = std::min<double>(report.targetSpeedMs, policy::getSafePolicy(policy::maxSpeedMs));
        totalTimeToReachTargetS = targetS / report.targetSpeedMs;
        report.recommendedTargetSpeed = report.targetSpeedMs;
    }
    else
    {
        /* Cost should be calculated as the time to target s:
         - the time to insertion
         - the ramp up/down time from current speed to lane speed
         - the current speed of the lane
         */

        mPredictor.getLaneSpeedAndTimeToInsertion(targetLane
                                                 ,report.targetSpeedMs
                                                 ,report.timeToInsertion
                                                 ,report.recommendedTargetSpeed);

        // If the car want to change 2 lanes, it needs to take into
        // consideration the time to change to the first lane
        int intermediateLane = 0;
        if (currentLane-targetLane >= 2)
        {
            intermediateLane = static_cast<int>(currentLane) - 1;
        }
        else if (currentLane-targetLane <= -2)
        {
            intermediateLane = static_cast<int>(currentLane) + 1;
        }

        report.timeToInsertion += reports[intermediateLane].timeToInsertion;
        report.recommendedTargetSpeed = std::min(reports[intermediateLane].recommendedTargetSpeed,
                                                 report.recommendedTargetSpeed);

#if VERBOSE > 1
        std::cout << "Time to insertion : \t" << report.timeToInsertion
                  << " seconds. Recommended insertion speed : \t " << report.recommendedTargetSpeed
                  << "\nTarget speed : " << report.targetSpeedMs << "\n";
#endif


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
        const double sameLaneDistance = report.recommendedTargetSpeed * report.timeToInsertion;

        // Sanity check. If I have to wait for too long, this is not a good choice
        if (sameLaneDistance >= 150)
        {
            costs[currentTargetLaneIndex] = 1.0;
            return;
        }

        // Distance travelled while ramping up or down the speed to lane speed
        constexpr static const double maximumSafeAcceleration = policy::getSafePolicy(policy::maxAccelerationMs);
        const double timeToReachLaneSpeed = fabs(report.recommendedTargetSpeed - currentSpeedMs) / maximumSafeAcceleration;
        // s = v * t + 0.5 * a * t * t
        const double rampDistance = (currentSpeedMs * timeToReachLaneSpeed) +
                        (0.5 * maximumSafeAcceleration * utl::sqr(timeToReachLaneSpeed));

        assert(rampDistance >= 0);

        // v = d / t or t = d / v
        double timeToReachTargetSAtFullSpeed = ((targetS - sameLaneDistance) - rampDistance) / report.targetSpeedMs;

#if VERBOSE > 2
        std::cout << "ramp distance " << rampDistance << "\n";
        std::cout << "timeToReachTargetSAtFullSpeed : " <<  timeToReachTargetSAtFullSpeed << "\n";
#endif

        // * If the car drives in the US, there are no specific lane goal.
        //   The prefered target lane is the current one in order to reduce
        //   the number of lane change. Switching lane is (slightly) penalised.
        // * If the car drives in EU for example, the should stay on the right
        //   most lane if the car is not undertaking. In that case the prefered
        //   target lane is the right most one. Not being on that lane will be
        //   slightly penalized.
        constexpr double changingLanePenaltyPercentage = 0.05;
        const double delta_d = (std::abs(preferedTargetLane - targetLane)
                                * changingLanePenaltyPercentage) + 1.0;

        // Penalty for changing lane, prevents over changing
        timeToReachTargetSAtFullSpeed *= delta_d;
        
        totalTimeToReachTargetS = report.timeToInsertion + timeToReachLaneSpeed + timeToReachTargetSAtFullSpeed;
    }

    const double minimumCostTime = targetS / policy::maxSpeedMs;
    assert(totalTimeToReachTargetS > minimumCostTime);

    double cost = utl::sigmoid(totalTimeToReachTargetS / minimumCostTime);
    costs[currentTargetLaneIndex] = cost;

#if VERBOSE > 1
    std::cout << "totalTimeToReachTargetS : " <<  totalTimeToReachTargetS << "\n";
    std::cout << "\t-->\tCost of lane : \t" << targetLane << "\t is : \t" << cost << "\n";
    std::cout << "%% Report Behavior : \t " << report.behavior << "\n";
    std::cout << "%% Report Speed : \t " << report.targetSpeedMs << "\n";
#endif
    
    // Recursivelly call for a cost estimation on the 2 neighboor lanes
    this->cost(currentLane,
               static_cast<Lane>(currentTargetLaneIndex - 1),
               preferedTargetLane,
               currentSpeedMs,
               warnings,
               lanes,
               mResults,
               costs);

    this->cost(currentLane,
               static_cast<Lane>(currentTargetLaneIndex + 1),
               preferedTargetLane,
               currentSpeedMs,
               warnings,
               lanes,
               mResults,
               costs);
}
