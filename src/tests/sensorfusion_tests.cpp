
#include <iostream>
#include <vector>
#include "gtest/gtest.h"

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

TEST_F(SensorFusionTest, GET_DISTANCE_AND_SPEED_CAR_AHEAD)
{
    std::vector<std::vector<double>> sensorData;
    const double vehicles_d = 6.0;
    const double myAV_d = 10.0;
    const double interCarDistance = 30.0;

    mSensorFusion.updateMyAVData(0, 0, myAV_d, vehicles_d, 0, 0);
    mSensorFusion.updateCarsData(sensorData);

    double distance;
    double speed;

    bool success = mSensorFusion.getDistanceAndSpeedCarAhead(distance, speed);

    ASSERT_FALSE(success);

    std::vector<double> someCar;
    someCar.push_back(0); // id
    someCar.push_back(0); // x
    someCar.push_back(0); // y
    someCar.push_back(0); // x_dot
    someCar.push_back(0); // y_dot
    someCar.push_back(myAV_d + interCarDistance); // s
    someCar.push_back(vehicles_d); // d
    sensorData.push_back(someCar);

    mSensorFusion.updateCarsData(sensorData);

    success = mSensorFusion.getDistanceAndSpeedCarAhead(distance, speed);

    ASSERT_TRUE(success);
    ASSERT_DOUBLE_EQ(distance, interCarDistance);
    ASSERT_DOUBLE_EQ(speed, 0.0);
}
