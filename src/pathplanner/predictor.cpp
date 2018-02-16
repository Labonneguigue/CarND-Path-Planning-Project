#include <iostream>
#include <limits>

#include "predictor.h"


Predictor::Predictor(SensorFusion& sensorFusion)
: mSensorFusion(sensorFusion)
{}

Predictor::~Predictor()
{}

const bool Predictor::anyWarnings(Predictor::Warnings &warnings) const
{
    bool anyWarningRaised = false;
    // First, I check for slow car ahead
    double distanceCarAhead;
    double speedCarAhead;
    if (mSensorFusion.getDistanceAndSpeedCarAhead(distanceCarAhead, speedCarAhead))
    {
        if (distanceCarAhead <= policy::detectionDistance)
        {
            std::cout << "Car ahead closer than detectionDistance : " << policy::detectionDistance << "\n";
            warnings.slowCarAhead = true;
            warnings.slowCarAheadSpeed = speedCarAhead;
            anyWarningRaised = true;
        }
    }

    // Second, I check whether there are cars crossing the planned path

    ///@todo
    {

    }

    return anyWarningRaised;
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
        else
        {
            std::cout << "Car too far, distance = " << utl::distance(mSensorFusion.myAV().x,
                                                                     mSensorFusion.myAV().y,
                                                                     cars[car].x,
                                                                     cars[car].y) << "\n";
        }
    }
}

void Predictor::getLaneSpeedAndTimeToInsertion(const int lane,
                                               double& laneSpeedMs,
                                               double& timeToInsertionS) const
{
    laneSpeedMs = utl::mph2ms(policy::maxSpeedMph);
    timeToInsertionS = 0;
    std::vector<DetectedVehicleData> carsInTargetLane;
    int indexClosestCar; //Index in the carsInTargetLane vector;
    double closestDistance = std::numeric_limits<double>::max();

    /* For each of the surrounding cars, I'll check only the cars in the
     considered lane. Among them, if they are too far, I discard them.
     Among the remaining ones, I find the slowest one below the speed
     I won't be able to exceed.
     */

    std::cout << " Cars detected in my lane " << lane << " : ";

    for (int car = 0; car < mNearbyCars.size() ; ++car)
    {
        if (mNearbyCars[car].lane == lane)
        {
            std::cout << "Car " << mNearbyCars[car].id << " s : " << mNearbyCars[car].s << " | ";
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
        // If indexLastCar has been set, at least one car has been detected
        // I now consider that I might have to wait before switching lane
        // To make this easy, I'll consider my speed to be constant
        std::cout << "Closest car is " << carsInTargetLane[indexClosestCar].id;
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
    else
    {
        std::cout << "None -> timeToInsertion is 0.\n";
    }
}

const bool Predictor::isCarTooFarBehind(const DetectedVehicleData car) const
{
    return true;
}

const bool Predictor::isCarTooFarAhead(const DetectedVehicleData car) const
{
    return false;
}
