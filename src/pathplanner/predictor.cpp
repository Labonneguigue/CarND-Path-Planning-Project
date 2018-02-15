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

void Predictor::prepareSensorDataForPrediction()
{
    std::vector<DetectedVehicleData> cars = mSensorFusion.detectedCars();
    mNearbyCars.clear();
    for (int car = 0; car < cars.size() ; ++car)
    {
        if (utl::distance(mSensorFusion.myAV().x,
                          mSensorFusion.myAV().y,
                          cars[car].x,
                          cars[car].y) < 100){
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
    int indexLastCar = -1;
    int lastCarInQueue_s = 0;

    /* For each of the surrounding cars, I'll check only the cars in the
     considered lane. Among them, if they are too far, I discard them.
     Among the remaining ones, I find the slowest one below the speed
     I won't be able to exceed.
     */
    for (int car = 0; car < mNearbyCars.size() ; ++car)
    {
        if (mNearbyCars[car].lane == lane)
        {
            if (mNearbyCars[car].speedMs < laneSpeedMs)
            {
                laneSpeedMs = mNearbyCars[car].speedMs;
                indexLastCar = car;
            }
            if (indexLastCar == -1)
            {
                lastCarInQueue_s = mNearbyCars[car].s;
                indexLastCar = car;
            }
            else if (mNearbyCars[car].s < lastCarInQueue_s)
            {
                lastCarInQueue_s = mNearbyCars[car].s;
                indexLastCar = car;
            }
        }
    }
    if (indexLastCar != -1)
    {
        // If indexLastCar has been set, at least one car has been detected
        // I now consider that I might have to wait before switching lane

        /// @todo implement that
        timeToInsertionS = 20;
    }
    else
    {
        std::cout << "No car detected on lane : " << lane << " -> timeToInsertion is 0.\n";
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
