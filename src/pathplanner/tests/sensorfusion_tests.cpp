
#include <iostream>
#include <vector>
#include "gtest/gtest.h"

#include "drivingpolicy.h"
#include "roadtypes.h"
#include "sensorfusion.h"


class SensorFusionTest : public ::testing::Test
{
public:

    SensorFusionTest()
    : mSensorFusion()
    {}

    virtual void SetUp() {
    }

    virtual void TearDown() {
        
    }

    SensorFusion mSensorFusion; ///< Collection of data about my car and the surrounding ones

    
};

TEST_F(SensorFusionTest, CarsOrderedAfterUpdate)
{
    // TESTSTEP: Creation of a few cars.
    // EXPECTED: I expect the cars to be sorted by ascending id.

    std::vector<std::vector<double>> sensorFusionData;
    double id = 0.0;
    double x = 0.0;
    double y = 0.0;
    double x_dot = 15.0;
    double y_dot = 0.0;
    double s = 20.0;

    double d = mSensorFusion.highway().getDFromLane<double>(firstLane);
    std::vector<double> leftCar = {id++,x, y, x_dot, y_dot, s, d, firstLane};

    d = mSensorFusion.highway().getDFromLane<double>(thirdLane);
    std::vector<double> rightCar = {id++,x, y, x_dot, y_dot, s, d, thirdLane};

    d = mSensorFusion.highway().getDFromLane<double>(secondLane);
    std::vector<double> carAhead = {id++,x, y, x_dot, y_dot, s + 30, d, secondLane};

    sensorFusionData.push_back(carAhead);
    sensorFusionData.push_back(leftCar);
    sensorFusionData.push_back(rightCar);

    mSensorFusion.updateCarsData(sensorFusionData);

    const std::vector<DetectedVehicleData> detectedVehicles = mSensorFusion.detectedCars();

    int previousId = 0;
    for (int car = 0 ; car < detectedVehicles.size() ; ++car)
    {
        if (car != 0)
        {
            ASSERT_GT(detectedVehicles[car].id, previousId);
        }
        previousId = detectedVehicles[car].id;
    }

    sensorFusionData.clear();

    sensorFusionData.push_back(rightCar);
    sensorFusionData.push_back(leftCar);
    sensorFusionData.push_back(carAhead);

    mSensorFusion.updateCarsData(sensorFusionData);
    const std::vector<DetectedVehicleData> lastDetectedVehicles = mSensorFusion.detectedCars();

    previousId = 0;
    for (int car = 0 ; car < lastDetectedVehicles.size() ; ++car)
    {
        if (car != 0)
        {
            ASSERT_GT(lastDetectedVehicles[car].id, previousId);
        }
        previousId = lastDetectedVehicles[car].id;
    }
}

