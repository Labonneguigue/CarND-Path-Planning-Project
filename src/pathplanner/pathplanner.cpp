
#include <iostream>
#include "pathplanner.h" // PathPlanner
#include "spline.h"

PathPlanner::PathPlanner(SensorFusion& sensorFusion,
                         BehaviorPlanner& behaviorPlanner,
                         TrajectoryGenerator& trajectoryGenerator)
: mSensorFusion(sensorFusion)
, mBehaviorPlanner(behaviorPlanner)
, mTrajectoryGenerator(trajectoryGenerator)
, mCurrentTargetVelocityMs(mMaximumVelocityMs)
{}

PathPlanner::~PathPlanner()
{}

void PathPlanner::solvePath(MapData mapData,
                            ControllerFeedback controllerFeedback,
                            std::vector<double>& next_x,
                            std::vector<double>& next_y){
    // Make sure the planned path is empty
    next_x.empty();
    next_y.empty();

    double distanceCarAhead;
    double speedCarAhead;
    if (mSensorFusion.getDistanceAndSpeedCarAhead(distanceCarAhead, speedCarAhead))
    {
        if (distanceCarAhead < 50)
        {
            std::cout << "Car ahead closer than 50\n";
            mCurrentTargetVelocityMs = speedCarAhead;
        }
    }
    else
    {
        mCurrentTargetVelocityMs = mMaximumVelocityMs;
    }

    // Vector of widely spaced waypoints to be interpolated with a spline
    // to smooth out the car trajectory
    std::vector<double> majorWayPoints_x;
    std::vector<double> majorWayPoints_y;

    const double remainingPathSize = controllerFeedback.remainingPath_x.size();

    // I need to extend the planned path. The following car x, y, and yaw coordinates
    // are usefull to that regard. They'll be different depending on whether the
    // remaining planned path not yet executed by the controller is small or not.
    double endPathCar_x;
    double endPathCar_y;
    double endPathCar_yaw;

    //std::cout << "Remaining path size : " << remainingPathSize << "\n";

    if (remainingPathSize < 2)
    {
        // If the remaing path is too small, I use the car as a starting reference
        endPathCar_x = mSensorFusion.myAV().x;
        endPathCar_y = mSensorFusion.myAV().y;
        endPathCar_yaw = utl::deg2rad(mSensorFusion.myAV().yaw);

        // By adding the previous car position and the current one, I make sure that the
        // generated trajectory will be smooth since the spline runs through every
        // provided points.
        majorWayPoints_x.push_back(endPathCar_x - cos(endPathCar_yaw));
        majorWayPoints_y.push_back(endPathCar_y - sin(endPathCar_yaw));
    }
    else
    {
        // I use the previous path. The points closest to the car are the ones at the
        // end of the remainingPath.
        // I
        endPathCar_x = controllerFeedback.remainingPath_x[remainingPathSize - 1];
        endPathCar_y = controllerFeedback.remainingPath_y[remainingPathSize - 1];

        const double previousreference_x = controllerFeedback.remainingPath_x[remainingPathSize - 2];
        const double previousreference_y = controllerFeedback.remainingPath_y[remainingPathSize - 2];

        // I need to calculate the ending vehicle orientation (yaw)
        endPathCar_yaw = atan2(endPathCar_y - previousreference_y,
                               endPathCar_x - previousreference_x);

        majorWayPoints_x.push_back(previousreference_x);
        majorWayPoints_y.push_back(previousreference_y);

        // I start by adding to the planned path the ones resulting from
        // previous iterations.
        next_x = controllerFeedback.remainingPath_x;
        next_y = controllerFeedback.remainingPath_y;

    }

    //std::cout << majorWayPoints_x[majorWayPoints_x.size()-2] << " " << majorWayPoints_y[majorWayPoints_y.size()-2] << "\n";
    //std::cout << majorWayPoints_x[majorWayPoints_x.size()-1] << " " << majorWayPoints_y[majorWayPoints_y.size()-1] << "\n";

    majorWayPoints_x.push_back(endPathCar_x);
    majorWayPoints_y.push_back(endPathCar_y);

    // Construction of 3 major waypoints at 30, 60 and 90 meters ahead of the car
    for (int wp = 30; wp <= 90 ; wp+= 30){
        std::vector<double> next_wp = utl::getXY<double>(mSensorFusion.myAV().s + wp,
                                                         utl::getDFromLane<double>(mSensorFusion.myAV().lane),
                                                         mapData.waypoints_s,
                                                         mapData.waypoints_x,
                                                         mapData.waypoints_y);
        majorWayPoints_x.push_back(next_wp[0]);
        majorWayPoints_y.push_back(next_wp[1]);

      /*  std::cout << wp << " " << majorWayPoints_x[majorWayPoints_x.size()-1]
                        << " " << majorWayPoints_y[majorWayPoints_y.size()-1] << "\n";*/
    }



    // Once I have my sparse trajectory made of waypoints, I change the
    // frame of reference so that the car is a (0, 0, 0).
    const double target_yaw = 0;
    for (int wp = 0 ; wp<majorWayPoints_x.size() ; ++wp)
    {
        double shift_x = majorWayPoints_x[wp] - endPathCar_x;
        double shift_y = majorWayPoints_y[wp] - endPathCar_y;

        majorWayPoints_x[wp] = (shift_x * cos(target_yaw-endPathCar_yaw)) -
                               (shift_y * sin(target_yaw-endPathCar_yaw));
        majorWayPoints_y[wp] = (shift_x * sin(target_yaw-endPathCar_yaw)) +
                               (shift_y * cos(target_yaw-endPathCar_yaw));
    }

    /*
    for (int i = 0 ; i< next_x.size() ; i++)
    {
        std::cout << next_x[i] << " " << next_y[i] << "\n";
    }*/

    tk::spline spline;

    // Provide the waypoints to the spline
    spline.set_points(majorWayPoints_x, majorWayPoints_y);

    const double target_x = 30.0;
    // The operator() returns the y coordinate of the point on the spline
    // having for x coordinate the provided argument.
    const double target_y = spline(target_x);

    const double target_distance = sqrt(utl::sqr(target_x) + utl::sqr(target_y));

    const double N = (target_distance / (0.02 * mCurrentTargetVelocityMs));
    const double x_increment = target_x / N;
    double x_position = 0;

    //std::cout << "Spline position: \n";

    for (int wp = 1; wp <= (mNumbersOfWaypoints-remainingPathSize) ; ++wp)
    {
        // We have the following equation
        // N * 0.02 * v = d
        // Each planned waypoints are supposed to be reached every 0.02 seconds
        // The spacing between them is therefore crucial.

        const double x = x_position + x_increment;
        const double y = spline(x);

        x_position = x;

        // Once this point is obtained, I modify its reference frame back to
        // the world reference frame.
        const double nextWayPoint_x = endPathCar_x + (x * cos(endPathCar_yaw)) - (y * sin(endPathCar_yaw));
        const double nextWayPoint_y = endPathCar_y + (x * sin(endPathCar_yaw)) + (y * cos(endPathCar_yaw));
        if (next_y.size() > 0)
            assert(utl::distance(nextWayPoint_x, nextWayPoint_y, next_x[next_x.size()-1], next_y[next_y.size()-1]) < 0.5);

        next_x.push_back(nextWayPoint_x);
        next_y.push_back(nextWayPoint_y);

        //std::cout << next_x[next_x.size()-1] << " " << next_y[next_y.size()-1] << "\n";
    }
}
