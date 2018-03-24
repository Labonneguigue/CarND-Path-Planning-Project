
#include <iostream>
#include <limits>

#include "predictor.h"

#define VERBOSE 2

Predictor::Predictor(SensorFusion& sensorFusion)
: mSensorFusion(sensorFusion)
, mMyAV(mSensorFusion.myAV())
, mCarsByLane(mSensorFusion.highway().getNumberLanes())
{}

Predictor::~Predictor()
{}

void Predictor::environmentalEvaluation(Predictor::Warnings &warnings)
{
    // Retrieves information from the Sensor Fusion module
    prepareSensorDataForPrediction();

    // First, I check for slow car ahead
    double distanceCarAhead;
    double speedCarAhead;
    if (getDistanceAndSpeedCarAhead(distanceCarAhead,speedCarAhead))
    {
#if VERBOSE > 1
        std::cout << "Car ahead closer than detectionDistance: "
                  << policy::detectionDistance << "\n";
#endif
        warnings.slowCarAhead = true;
        warnings.slowCarAheadSpeed = speedCarAhead;
        warnings.anyWarningRaised = true;
    }

    // Second, I check whether there are cars crossing the planned path
    std::vector<DetectedVehicleData> cars = mSensorFusion.detectedCars();
    for (unsigned int car = 0; car < cars.size() ; ++car)
    {
        if (cars[car].isChangingLane)
        {
            warnings.carCrossingPlannedPath = true;
            // Early termination, the Behavior will be called
            // to assess the situation
            return;
        }
    }
}

void Predictor::prepareSensorDataForPrediction()
{
    // Retrieve all surrounding cars detected by the sensor fusion module
    std::vector<DetectedVehicleData> cars = mSensorFusion.detectedCars();

    // Reinitialise vector of cars sorted by lanes
    mCarsByLane = std::vector<std::vector<DetectedVehicleData>>(mSensorFusion.highway().getNumberLanes());

    for (unsigned int car = 0; car < cars.size() ; ++car)
    {
        if (utl::distance(mSensorFusion.myAV().x,
                          mSensorFusion.myAV().y,
                          cars[car].x,
                          cars[car].y) < policy::detectionDistance){
            mCarsByLane[static_cast<int>(cars[car].lane)].push_back(cars[car]);
        }
#if VERBOSE > 4
        else
        {
            std::cout << "Car too far, distance = "
                      << utl::distance(mSensorFusion.myAV().x,
                                       mSensorFusion.myAV().y,
                                       cars[car].x,
                                       cars[car].y)
                      << "\n";
        }
#endif
    }
#if VERBOSE > 2
    std::cout << "Size of 3 car lanes : "
              << mCarsByLane[0].size() << " "
              << mCarsByLane[1].size() << " "
              << mCarsByLane[2].size() << "\n";
#endif
}

bool Predictor::getDistanceAndSpeedCarAhead(double& distance,
                                            double& speed)
{
    bool found = false;
    speed = std::numeric_limits<double>::max();
    distance = std::numeric_limits<double>::max();
    const std::vector<DetectedVehicleData>& vehiclesInMyLane = mCarsByLane[mMyAV.lane];
    for (unsigned int car = 0; car < vehiclesInMyLane.size() ; car++)
    {
        // If the car is in my lane and its s is greater than mine,
        // I consider it
        double signedDistance = vehiclesInMyLane[car].s - mMyAV.s;
        if ((signedDistance > 0.0) &&
            (signedDistance <= policy::detectionDistance))
        {
            // If the distance between us is smaller than previously
            // recorded, it becomes the closest car to me.
            distance = signedDistance;
            speed = sqrt(utl::sqr(vehiclesInMyLane[car].x_dot)
                         + utl::sqr(vehiclesInMyLane[car].y_dot));
            found = true;
            assert(distance > 0);
#if VERBOSE > 2
            std::cout << " ** Distance car ahead " << mCars[car].id << " is " << speed << "\n";
#endif
        }
    }
    return found;
}

void Predictor::getLaneSpeedAndTimeToInsertion(const int lane,
                                               double& laneSpeedMs,
                                               double& timeToInsertionS,
                                               double& recommendedSpeedMs) const
{
    laneSpeedMs = policy::getSafePolicy(policy::maxSpeedMs);
    timeToInsertionS = -1; // If not set, assert should fail

    // Vehicles in the lane sorted by position on the road
    // First being behind and the last being ahead.
    std::vector<DetectedVehicleData> carsInTargetLane = mCarsByLane[lane];
    std::sort(carsInTargetLane.begin(), carsInTargetLane.end());

#if VERBOSE > 1
    std::cout << "Lane car's S : ";
    for (const DetectedVehicleData& car: carsInTargetLane)
    {
        std::cout << car.s << " ";
    }
    std::cout << "\n";
#endif

    int indexClosestCar = -1; //Index in the carsInTargetLane vector;
    double closestDistance = std::numeric_limits<double>::max();

    /* For each of the surrounding cars, I'll check only the cars in the
     considered lane. Among them, if they are too far, I discard them.
     Among the remaining ones, I find the slowest one below the speed
     I won't be able to exceed.
     */

    if (carsInTargetLane.empty())
    {
        timeToInsertionS = 0.0;
        recommendedSpeedMs = policy::getSafePolicy(policy::maxSpeedMs);
#if VERBOSE > 1
        std::cout << " -- None -> timeToInsertion is 0.\n";
#endif
    }
    else
    {
        bool safeToChangeLane = true;

        for (unsigned int car = 0 ; car < carsInTargetLane.size() ; ++car)
        {
            const double distance = fabs(carsInTargetLane[car].s
                                         - mSensorFusion.myAV().s);

            if (distance < closestDistance)
            {
                closestDistance = distance;
                indexClosestCar = car;
            }

            if (isThisCarBlockingMe(carsInTargetLane[car],
                                    mSensorFusion.myAV().s))
            {
                safeToChangeLane = false;
#if VERBOSE > 1
                std::cout << "[[[ CAR BLOCKING ME ]]]\n";
#endif
            }

            if (carsInTargetLane[car].speedMs < laneSpeedMs)
            {
                laneSpeedMs = carsInTargetLane[car].speedMs;
            }
        }

#if VERBOSE > 1
        std::cout << "Lane speed : " << laneSpeedMs << "\n";
#endif

        if (laneSpeedMs < 0.5)
        {
            timeToInsertionS = std::numeric_limits<double>::max();
        }
        else
        {
            if (safeToChangeLane)
            {
#if VERBOSE > 1
                std::cout << " -- Safe to change lane : " << lane << "\n";
#endif
                timeToInsertionS = 0;
            }
            else
            {
                // If indexLastCar has been set, at least one car has been detected
                // I now consider that I might have to wait before switching lane
                // To make this easy, I'll consider my speed to be constant
                assert(indexClosestCar >= 0);

#if VERBOSE > 1
                std::cout << " -- Closest car is "
                          << carsInTargetLane[indexClosestCar].id
                          << " with distance : "
                          << closestDistance << "\n";
#endif

                double frontInsertionTime = std::numeric_limits<double>::max();

                double rearInsertionTime = std::numeric_limits<double>::max();
                const double slowDownRatio = 0.75;
                const double slowSpeedForReadInsertion = slowDownRatio * laneSpeedMs;

                if (0)//(carsInTargetLane[indexClosestCar].speedMs < mSensorFusion.myAV().speedMs))
                {
                    //I'll try to change lane in front of that car.
                    const double delta_s = mSensorFusion.myAV().s - carsInTargetLane[indexClosestCar].s;
                    frontInsertionTime = (policy::safeDistance - delta_s) /
                    (mSensorFusion.myAV().speedMs - carsInTargetLane[indexClosestCar].speedMs);
#if VERBOSE > 1
                    std::cout << " : Front insertion possible in : " << timeToInsertionS << "\n";
#endif
                }
                else
                {
                    double positionSforInsertion = (carsInTargetLane[indexClosestCar].s
                                                    - policy::safeDistance);
                    int carIndex = indexClosestCar;
                    bool insertionSlotFound = false;
                    while ((carIndex >= 0) && !insertionSlotFound)
                    {
                        if (!isThisCarBlockingMe(carsInTargetLane[carIndex], positionSforInsertion))
                        {
                            insertionSlotFound = true;
                        }
                        else
                        {
                            positionSforInsertion -= policy::safeDistance;
                            --carIndex;
                        }
                    }


                    if (insertionSlotFound)
                    {
                        const double delta_s = carsInTargetLane[indexClosestCar].s
                                                - mSensorFusion.myAV().s;
                        const double delta_speedMs = carsInTargetLane[indexClosestCar].speedMs
                                                    - slowSpeedForReadInsertion;
                        rearInsertionTime =  (policy::safeDistance - delta_s) / delta_speedMs;
#if VERBOSE > 1
                        std::cout << "Rear insertion possible " << positionSforInsertion
                        << " meters behind in " << rearInsertionTime << " seconds.\n";
#endif
                    }
                    else
                    {
                        rearInsertionTime = 20;
#if VERBOSE > 1
                        std::cout << "Rear insertion not possible.\n";
#endif
                    }


                }

                if (frontInsertionTime < rearInsertionTime)
                {
                    recommendedSpeedMs = policy::getSafePolicy(policy::maxSpeedMs);
                    timeToInsertionS = frontInsertionTime;
                }
                else
                {
                    recommendedSpeedMs = slowSpeedForReadInsertion;
                    timeToInsertionS = rearInsertionTime;
                }

                assert(timeToInsertionS > 0.0);
            }
        }
    }

    assert(timeToInsertionS >= 0.0);
    assert(recommendedSpeedMs >= 0.0); // I don't support backing up for now
    assert(laneSpeedMs >= 0.0); // I don't support detection of cars backing up
}

bool Predictor::canIChangeLane(const Lane targetLane, const double positionS)
{
    prepareSensorDataForPrediction();

    std::vector<DetectedVehicleData> carsInTargetLane = mCarsByLane[static_cast<int>(targetLane)];

    for (const DetectedVehicleData car : carsInTargetLane)
    {
        if (isThisCarBlockingMe(car, positionS)) return false;
    }
    return true;
}
