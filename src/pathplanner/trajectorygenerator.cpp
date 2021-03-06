#include <cmath>
#include "spline.h"
#include "trajectorygenerator.h"

#define VERBOSE 2

TrajectoryGenerator::TrajectoryGenerator(SensorFusion& sensorFusion,
                                         Predictor& predictor,
                                         MapData& mapData)
: mSensorFusion(sensorFusion) //remove if not used
, mPredictor(predictor)
, mMapData(mapData)
, mMajorWayPoints_x(0)
, mMajorWayPoints_y(0)
, mPlannedTrajectory_x(0)
, mPlannedTrajectory_y(0)
, mEndPathCar(0, 0, 0, 0, 0, 0)
, mRemainingPathSize(0)
, mCurrentTargetVelocityMs(mMaximumVelocityMs)
, mCurrentTargetLane(undefined)
, mSpeedRegulator()
, mChangingLane(false)
{
    // Initialisation of the PID to regulate the speed when I am following a car
    mSpeedRegulator.init(0.05, 1, 0.0005);
}

TrajectoryGenerator::~TrajectoryGenerator()
{}

void TrajectoryGenerator::computeTrajectory(const ControllerFeedback& controllerFeedback,
                                            const BehaviorPlanner::HighLevelTrajectoryReport& result,
                                            std::vector<double>& next_x,
                                            std::vector<double>& next_y)
{
    // I initialise the path to the remaining one
    initialiseTrajectory(controllerFeedback);

    // If I was changing lane, I check if it is over now
    if (mChangingLane
        && !mSensorFusion.highway().ongoingLaneChange(mSensorFusion.myAV().d))
    {
        mChangingLane = false;
    }

    // Process the Trajectory Report
    setCurrentTargetVelocity(result.targetSpeedMs);

    if (!mChangingLane){

        bool acceptingNewTargetLane = false;

        if (result.behavior == BehaviorPlanner::keepLane)
        {
            mCurrentTargetLane = result.targetLane;
            setCurrentTargetVelocity(result.targetSpeedMs);
        }
        else if (result.behavior == BehaviorPlanner::rightLaneChange
              || result.behavior == BehaviorPlanner::leftLaneChange)
        {
            acceptingNewTargetLane = true;
        }
        else if (result.behavior == BehaviorPlanner::prepareLeftLaneChange
              || result.behavior == BehaviorPlanner::prepareRightLaneChange)
        {
            double laneSpeedMs;
            double timeToInsertion;
            double recommendedTargetSpeed;
            mPredictor.getLaneSpeedAndTimeToInsertion(result.targetLane,
                                                      laneSpeedMs,
                                                      timeToInsertion,
                                                      recommendedTargetSpeed);

            if (timeToInsertion <= 0.1)
            {
                acceptingNewTargetLane = true;
                setCurrentTargetVelocity(result.targetSpeedMs);
            }
            else
            {
                setCurrentTargetVelocity(result.recommendedTargetSpeed);
            }
        }

        if (acceptingNewTargetLane)
        {
            if (result.targetLane > mCurrentTargetLane)
            {
                if (mPredictor.canIChangeLane(static_cast<Lane>(mCurrentTargetLane + 1), mSensorFusion.myAV().s))
                {
                    mCurrentTargetLane = static_cast<Lane>(mCurrentTargetLane + 1);
                }
            }
            else if (result.targetLane < mCurrentTargetLane)
            {
                if (mPredictor.canIChangeLane(static_cast<Lane>(mCurrentTargetLane - 1), mSensorFusion.myAV().s))
                {
                    mCurrentTargetLane = static_cast<Lane>(mCurrentTargetLane - 1);
                }
            }
            mChangingLane = true;
        }
    }

#if VERBOSE > 2
    std::cout << "Result.targetLane: " << result.targetLane << "  Target lane : " << mCurrentTargetLane << "\n";
#endif

#if VERBOSE > 0
    std::cout << "TrajectoryGenerator targets : speed \t" << mCurrentTargetVelocityMs << "\tlane :\t" << mCurrentTargetLane << "\n'";
#endif

    // Construction of 3 major waypoints at 50, 100 and 150 meters ahead of the car
    const double step = 50.0;
    for (double wp = step; wp <= 3*step ; wp+= step)
    {
        std::vector<double> next_wp = utl::getXY<double>(mSensorFusion.myAV().s + wp,
                                                         mSensorFusion.highway().getDFromLane<double>(mCurrentTargetLane),
                                                         mMapData.waypoints_s,
                                                         mMapData.waypoints_x,
                                                         mMapData.waypoints_y);
        mMajorWayPoints_x.push_back(next_wp[0]);
        mMajorWayPoints_y.push_back(next_wp[1]);
    }

    // Once I have my sparse trajectory made of waypoints, I change the
    // frame of reference so that the car is a (0, 0, 0).
    const double target_yaw = 0;
    for (unsigned int wp = 0U ; wp<mMajorWayPoints_x.size() ; ++wp)
    {
        double shift_x = mMajorWayPoints_x[wp] - mEndPathCar.x;
        double shift_y = mMajorWayPoints_y[wp] - mEndPathCar.y;

        mMajorWayPoints_x[wp] = (shift_x * cos(target_yaw-mEndPathCar.yaw)) -
        (shift_y * sin(target_yaw-mEndPathCar.yaw));
        mMajorWayPoints_y[wp] = (shift_x * sin(target_yaw-mEndPathCar.yaw)) +
        (shift_y * cos(target_yaw-mEndPathCar.yaw));
    }

    tk::spline spline;

    // Provide the waypoints to the spline
    spline.set_points(mMajorWayPoints_x, mMajorWayPoints_y);

    const double target_x = 30.0;
    // The operator() returns the y coordinate of the point on the spline
    // having for x coordinate the provided argument.
    const double target_y = spline(target_x);

    const double target_distance = sqrt(utl::sqr(target_x) + utl::sqr(target_y));
    double x_position = 0;

    //std::cout << "Spline position: \n";

    for (int wp = 1; wp <= (mNumbersOfWaypoints-mRemainingPathSize) ; ++wp)
    {
        // We have the following equation
        // N * 0.02 * v = d
        // Each planned waypoints are supposed to be reached every 0.02 seconds
        // The spacing between them is therefore crucial.
        computeStepSpeed();
        const double N = (target_distance / (mSimulatorWaypointsDeltaT * mEndPathCar.speedMs));
        const double delta_d = target_x / N;

        const double x = x_position + delta_d;
        const double y = spline(x);

        x_position = x;

        // Once this point is obtained, I modify its reference frame back to
        // the world reference frame.
        const double nextWayPoint_x = mEndPathCar.x + (x * cos(mEndPathCar.yaw)) - (y * sin(mEndPathCar.yaw));
        const double nextWayPoint_y = mEndPathCar.y + (x * sin(mEndPathCar.yaw)) + (y * cos(mEndPathCar.yaw));
        if (next_y.size() > 0)
        {
            if (utl::distance(nextWayPoint_x, nextWayPoint_y, next_x[next_x.size()-1], next_y[next_y.size()-1]) > 0.5)
            {
#if VERBOSE > 0
                std::cout << "WARNING : " << utl::distance(nextWayPoint_x, nextWayPoint_y, next_x[next_x.size()-1], next_y[next_y.size()-1]) << "\n";
#endif
            }
        }

        mPlannedTrajectory_x.push_back(nextWayPoint_x);
        mPlannedTrajectory_y.push_back(nextWayPoint_y);

        //std::cout << next_x[next_x.size()-1] << " " << next_y[next_y.size()-1] << "\n";
    }

    // Return result;
    next_x = mPlannedTrajectory_x;
    next_y = mPlannedTrajectory_y;
}

void TrajectoryGenerator::initialiseTrajectory(const ControllerFeedback& controllerFeedback)
{
    // Make sure the planned path is empty
    mPlannedTrajectory_x.empty();
    mPlannedTrajectory_y.empty();

    mMajorWayPoints_x.clear();
    mMajorWayPoints_y.clear();

    mRemainingPathSize = controllerFeedback.remainingPath_x.size();

    //std::cout << "Remaining path size : " << mRemainingPathSize << "\n";

    if (mRemainingPathSize < 2)
    {
        // If the remaing path is too small, I use the car as a starting reference
        mEndPathCar.x = mSensorFusion.myAV().x;
        mEndPathCar.y = mSensorFusion.myAV().y;
        mEndPathCar.yaw = utl::deg2rad(mSensorFusion.myAV().yaw);

        // By adding the previous car position and the current one, I make sure that the
        // generated trajectory will be smooth since the spline runs through every
        // provided points.
        mMajorWayPoints_x.push_back(mEndPathCar.x - cos(mEndPathCar.yaw));
        mMajorWayPoints_y.push_back(mEndPathCar.y - sin(mEndPathCar.yaw));
    }
    else
    {
        // I use the previous path. The points closest to the car are the ones at the
        // end of the remainingPath.
        // I
        mEndPathCar.x = controllerFeedback.remainingPath_x[mRemainingPathSize - 1];
        mEndPathCar.y = controllerFeedback.remainingPath_y[mRemainingPathSize - 1];

        const double previousreference_x = controllerFeedback.remainingPath_x[mRemainingPathSize - 2];
        const double previousreference_y = controllerFeedback.remainingPath_y[mRemainingPathSize - 2];

        // I need to calculate the ending vehicle orientation (yaw)
        mEndPathCar.yaw = atan2(mEndPathCar.y - previousreference_y,
                                mEndPathCar.x - previousreference_x);

        mMajorWayPoints_x.push_back(previousreference_x);
        mMajorWayPoints_y.push_back(previousreference_y);

        // The beginning of the planned path will be the remaining ones from the
        // last trajectory generation.
        mPlannedTrajectory_x = controllerFeedback.remainingPath_x;
        mPlannedTrajectory_y = controllerFeedback.remainingPath_y;

    }

#if VERBOSE > 2
    std::cout << mMajorWayPoints_x[mMajorWayPoints_x.size()-2] << " " << mMajorWayPoints_y[mMajorWayPoints_y.size()-2] << "\n";
    std::cout << mMajorWayPoints_x[mMajorWayPoints_x.size()-1] << " " << mMajorWayPoints_y[mMajorWayPoints_y.size()-1] << "\n";
#endif

    mMajorWayPoints_x.push_back(mEndPathCar.x);
    mMajorWayPoints_y.push_back(mEndPathCar.y);
    
}

void TrajectoryGenerator::computeStepSpeed()
{
    assert(mEndPathCar.speedMs >= 0.0);

    double distanceCarAhead;
    double speedCarAhead;
    double pidCorrectedSpeed = 0.0;

    if (mPredictor.getDistanceAndSpeedCarAhead(distanceCarAhead,
                                           speedCarAhead))
    {
        if (distanceCarAhead < policy::safeDistance * 1.5)
        {
            // I compute the Distance Error as the delta between the optimal
            // safe distance and the actual distance between us
            double deltaDistance = policy::safeDistance - distanceCarAhead;
            mSpeedRegulator.updateError(deltaDistance);
            pidCorrectedSpeed = mSpeedRegulator.totalError();
        }
    }

    double appliedSpeed = mCurrentTargetVelocityMs + pidCorrectedSpeed;

    if ( (std::abs(appliedSpeed - mEndPathCar.speedMs)
          / mSimulatorWaypointsDeltaT) > mMaximumAccelerationMs )
    {
        if (appliedSpeed > mEndPathCar.speedMs)
        {
            mEndPathCar.speedMs += mMaximumSpeedIncrement;
        }
        else
        {
            mEndPathCar.speedMs -= mMaximumSpeedIncrement;
        }
    }
    else
    {
        mEndPathCar.speedMs = appliedSpeed;
    }

    assert(mEndPathCar.speedMs <= policy::maxSpeedMs);
    std::cout << " Updated speed : "
              << mEndPathCar.speedMs
              << " current target velocity : "
              << mCurrentTargetVelocityMs << "\n";
}
