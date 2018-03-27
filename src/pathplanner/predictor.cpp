#include <iostream>
#include <limits>

#include "predictor.h"

#define DEBUG 0

Predictor::Predictor(SensorFusion& sensorFusion)
: mSensorFusion(sensorFusion)
, mNearbyCars()
{}

Predictor::~Predictor()
{}

void Predictor::environmentalEvaluation(Predictor::Warnings &warnings) const
{
    // First, I check for slow car ahead
    double distanceCarAhead;
    double speedCarAhead;
    if (mSensorFusion.getDistanceAndSpeedCarAhead(distanceCarAhead, speedCarAhead))
    {
        if (distanceCarAhead <= policy::detectionDistance)
        {
#if DEBUG
            std::cout << "Car ahead closer than detectionDistance : " << policy::detectionDistance << "\n";
#endif
            warnings.slowCarAhead = true;
            warnings.slowCarAheadSpeed = speedCarAhead;
            warnings.anyWarningRaised = true;
        }
    }

    // Second, I check whether there are cars crossing the planned path
    std::vector<DetectedVehicleData> cars = mSensorFusion.detectedCars();
    for (int car = 0; car < cars.size() ; ++car)
    {
        if (cars[car].isChangingLane)
        {
            warnings.carCrossingPlannedPath = true;
            // Early termination, the Behavior will be called and assess the situation
            return;
        }
    }
}

void Predictor::prepareSensorDataForPrediction()
{
    std::vector<DetectedVehicleData> cars = mSensorFusion.detectedCars();
    mNearbyCars.clear();
    for (int car = 0; car < cars.size() ; ++car)
    {
        if (utl::distance(mSensorFusion.myAV().x,
                          mSensorFusion.myAV().y,
                          cars[car].x,
                          cars[car].y) < mMaximumDetectionDistance){
            mNearbyCars.push_back(cars[car]);
        }
        #if DEBUG
        else
        {
            std::cout << "Car too far, distance = " << utl::distance(mSensorFusion.myAV().x,
                                                                     mSensorFusion.myAV().y,
                                                                     cars[car].x,
                                                                     cars[car].y) << "\n";
        }
        #endif
    }
}

void Predictor::getLaneSpeedAndTimeToInsertion(const int lane,
                                               double& laneSpeedMs,
                                               double& timeToInsertionS) const
{
    laneSpeedMs = utl::mph2ms(policy::maxSpeedMph);
    timeToInsertionS = 0;
    std::vector<DetectedVehicleData> carsInTargetLane;
    int indexClosestCar = -1; //Index in the carsInTargetLane vector;
    double closestDistance = std::numeric_limits<double>::max();

    /* For each of the surrounding cars, I'll check only the cars in the
     considered lane. Among them, if they are too far, I discard them.
     Among the remaining ones, I find the slowest one below the speed
     I won't be able to exceed.
     */
#if DEBUG
    std::cout << " -- Cars detected in my lane " << lane << " : ";
#endif

    for (int car = 0; car < mNearbyCars.size() ; ++car)
    {
        if (mNearbyCars[car].lane == lane)
        {
            std::cout << " -- Car " << mNearbyCars[car].id << " s : " << mNearbyCars[car].s << " speed : " <<  mNearbyCars[car].speedMs << " s_dot : " <<  mNearbyCars[car].s_dot << " | ";
            carsInTargetLane.push_back(mNearbyCars[car]);
            const double distance = utl::distance(mNearbyCars[car].x,
                                                  mNearbyCars[car].y,
                                                  mSensorFusion.myAV().x,
                                                  mSensorFusion.myAV().y);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                indexClosestCar = carsInTargetLane.size()-1;
            }
        }
    }
    if (!carsInTargetLane.empty())
    {
        std::cout << "\n";
        bool safeToChangeLane = true;

        for (int car = 0 ; car < carsInTargetLane.size() ; ++car)
        {
            if ((carsInTargetLane[car].s < (mSensorFusion.myAV().s + policy::safeDistance))
                && (carsInTargetLane[car].s >(mSensorFusion.myAV().s - policy::safeDistance)))
            {
                safeToChangeLane = false;
                std::cout << " -- Car blocking me\n";
            }

            if (carsInTargetLane[car].s > (mSensorFusion.myAV().s - policy::safeDistance) &&
                (carsInTargetLane[car].speedMs < laneSpeedMs))
            {
                laneSpeedMs = carsInTargetLane[car].speedMs;
            }

        }
        if (safeToChangeLane)
        {
            std::cout << " -- Safe to change lane : " << lane << "\n";
            timeToInsertionS = 0;
        }
        else
        {
            // If indexLastCar has been set, at least one car has been detected
            // I now consider that I might have to wait before switching lane
            // To make this easy, I'll consider my speed to be constant
            assert(indexClosestCar >= 0);
            std::cout << " -- Closest car is " << carsInTargetLane[indexClosestCar].id;
            if (carsInTargetLane[indexClosestCar].speedMs < mSensorFusion.myAV().speedMs)
            {
                //I'll try to change lane in front of that car.
                const double delta_s = mSensorFusion.myAV().s - carsInTargetLane[indexClosestCar].s;
                timeToInsertionS = (policy::safeDistance - delta_s) /
                (mSensorFusion.myAV().speedMs - carsInTargetLane[indexClosestCar].speedMs);
                std::cout << " : insertion possible in : " << timeToInsertionS << "\n";
            }
            else
            {
                std::cout << "\n";
                timeToInsertionS = 20;
            }
        }
    }
    else
    {
        std::cout << " -- None -> timeToInsertion is 0.\n";
    }
}

const bool Predictor::isCarTooFarBehind(const DetectedVehicleData car) const
{
    ///@todo: Implement and use
    return true;
}

const bool Predictor::isCarTooFarAhead(const DetectedVehicleData car) const
{
    ///@todo: Implement and use
    return false;
}
