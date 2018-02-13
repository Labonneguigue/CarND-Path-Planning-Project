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
            anyWarningRaised = true;
        }
    }

    // Second, I check whether there are cars crossing the planned path

    ///@todo
    {

    }

    return anyWarningRaised;
}

void Predictor::getLaneSpeedAndTimeToInsertion(const int lane,
                                               double& laneSpeed,
                                               double& timeToInsertion) const
{
    laneSpeed = policy::getSafePolicy(policy::maxSpeedMph);
    timeToInsertion = 0;
    std::vector<DetectedVehicleData> cars = mSensorFusion.detectedCars();
    int indexLastCar = -1;
    int lastCarInQueue_s = 0;
    for (int car = 0; car < cars.size() ; ++car)
    {
        if (cars[car].lane == lane)
        {
            if (!isCarTooFarBehind(cars[car]) || !isCarTooFarAhead(cars[car]))
            {
                if (cars[car].speed < laneSpeed)
                {
                    laneSpeed = cars[car].speed;
                    indexLastCar = car;
                    lastCarInQueue_s = cars[car].s;
                }
            }
        }
    }
    if (indexLastCar != -1)
    {
        // If indexLastCar has been set, at least one car has been detected
        // I now consider that I might have to wait before switching lane

        /// @todo implement that
        timeToInsertion = 20;
    }
    else
    {
        std::cout << "No car detected on lane : " << lane << "\n";
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
