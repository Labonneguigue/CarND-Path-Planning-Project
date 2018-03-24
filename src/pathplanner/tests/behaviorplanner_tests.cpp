
#include <iostream>
#include <vector>
#include "gtest/gtest.h"

#include "behaviorplanner.h"


class BehaviorPlannerTest : public ::testing::Test
{
public:

    BehaviorPlannerTest()
    : mSensorFusion()
    , mPredictor(mSensorFusion)
    , mBehaviorPlanner(mPredictor, mSensorFusion)
    {}

    virtual void SetUp();

    SensorFusion mSensorFusion;
    Predictor mPredictor;
    BehaviorPlanner mBehaviorPlanner;
};

void BehaviorPlannerTest::SetUp()
{
}

TEST_F(BehaviorPlannerTest, RightLaneChangeDueToCarAhead)
{
    std::vector<std::vector<double>> sensorFusionData;
    double id = 0;
    double x = 0;
    double y = 0;
    double x_dot = 0;
    double y_dot = 0;
    double s = 20;
    Lane lane = firstLane;
    double d = mSensorFusion.highway().getDFromLane<double>(lane);
    std::vector<double> car = {id++,x, y, x_dot, y_dot, s, d, lane};

    sensorFusionData.push_back(car);
    mSensorFusion.updateCarsData(sensorFusionData);

    mSensorFusion.updateMyAVData(x, y, 0, d, 0.0, 20);

    const BehaviorPlanner::HighLevelTrajectoryReport result = mBehaviorPlanner.computeNewTrajectory(Predictor::Warnings());

    ASSERT_EQ(result.behavior, BehaviorPlanner::rightLaneChange);
}

TEST_F(BehaviorPlannerTest, LeftLaneChangeDueToCarAhead)
{
    // Same test as above on the other side: Myself and a car ahead on the rightmost lane
    std::vector<std::vector<double>> sensorFusionData;
    double id = 0;
    double x = 0;
    double y = 0;
    double x_dot = 0;
    double y_dot = 0;
    double s = 20;
    Lane lane = thirdLane;
    double d = mSensorFusion.highway().getDFromLane<double>(lane);
    std::vector<double> car = {id++,x, y, x_dot, y_dot, s, d, lane};

    sensorFusionData.push_back(car);
    mSensorFusion.updateCarsData(sensorFusionData);

    mSensorFusion.updateMyAVData(x, y, 0, d, 0.0, 20);

    const BehaviorPlanner::HighLevelTrajectoryReport result = mBehaviorPlanner.computeNewTrajectory(Predictor::Warnings());

    ASSERT_EQ(result.behavior, BehaviorPlanner::leftLaneChange);
}

TEST_F(BehaviorPlannerTest, NoCars_ShouldKeepLane)
{
    // With 3 different speeds and no cars, I should stay on the same lane
    // My targetSpeed should be the maximum allowed speed
    const double x = 0;
    const double y = 0;
    const double s = 20;
    const double d = mSensorFusion.highway().getDFromLane<double>(secondLane);
    const double yaw = 0.0;
    double speed = 0.0;
    mSensorFusion.updateMyAVData(x, y, s, d, yaw, speed);

    const BehaviorPlanner::HighLevelTrajectoryReport result = mBehaviorPlanner.computeNewTrajectory();

    ASSERT_EQ(result.behavior, BehaviorPlanner::keepLane);
    ASSERT_FLOAT_EQ(result.targetSpeedMs, policy::getSafePolicy(policy::maxSpeedMs));
    ASSERT_FLOAT_EQ(result.targetSpeedMs, result.recommendedTargetSpeed);

    // Step 2 : Speed just bellow maximum safe speed
    speed = 20.0;
    mSensorFusion.updateMyAVData(x, y, s, d, yaw, speed);

    const BehaviorPlanner::HighLevelTrajectoryReport result2 = mBehaviorPlanner.computeNewTrajectory();

    ASSERT_EQ(result2.behavior, BehaviorPlanner::keepLane);
    ASSERT_FLOAT_EQ(result2.targetSpeedMs, policy::getSafePolicy(policy::maxSpeedMs));
    ASSERT_FLOAT_EQ(result.targetSpeedMs, result.recommendedTargetSpeed);

    // Step 3 : Speed above speed limit if that case might happens for some reason
    speed = 21.0;
    mSensorFusion.updateMyAVData(x, y, s, d, yaw, speed);

    const BehaviorPlanner::HighLevelTrajectoryReport result3 = mBehaviorPlanner.computeNewTrajectory();

    ASSERT_EQ(result3.behavior, BehaviorPlanner::keepLane);
    ASSERT_FLOAT_EQ(result3.targetSpeedMs, policy::getSafePolicy(policy::maxSpeedMs));
    ASSERT_FLOAT_EQ(result.targetSpeedMs, result.recommendedTargetSpeed);
}

TEST_F(BehaviorPlannerTest, SlowDownSinceChangingLaneIsImpossible)
{
    // Same test as above on the other side: Myself and a car ahead on the center lane, a car on both sides at the same d
    // All cars at 15 m/s - Myself at 20 m/s

    /*
    |   |   |   |
    |   | * |   |
    |   |   |   |
    | * | $ | * |
    |   |   |   |
    |   |   |   |
     */
    std::vector<std::vector<double>> sensorFusionData;
    double id = 0.0;
    double x = 0.0;
    double y = 0.0;
    double x_dot = 15.0;
    double y_dot = 0.0;
    double s = 20.0;

    double d = mSensorFusion.highway().getDFromLane<double>(firstLane);
    std::vector<double> leftCar = {id++,x, y, x_dot, y_dot, s, d, firstLane};
    sensorFusionData.push_back(leftCar);

    d = mSensorFusion.highway().getDFromLane<double>(thirdLane);
    std::vector<double> rightCar = {id++,x, y, x_dot, y_dot, s, d, thirdLane};
    sensorFusionData.push_back(rightCar);

    d = mSensorFusion.highway().getDFromLane<double>(secondLane);
    std::vector<double> carAhead = {id++,x, y, x_dot, y_dot, s + 30, d, secondLane};
    sensorFusionData.push_back(carAhead);

    mSensorFusion.updateCarsData(sensorFusionData);
    mSensorFusion.updateMyAVData(x, y, s, d, 0.0, 20); // 20 mph

    const BehaviorPlanner::HighLevelTrajectoryReport result = mBehaviorPlanner.computeNewTrajectory();

    EXPECT_EQ(result.behavior, BehaviorPlanner::keepLane);
    EXPECT_FLOAT_EQ(result.targetSpeedMs, x_dot);
    EXPECT_FLOAT_EQ(result.targetSpeedMs, result.recommendedTargetSpeed);
    
    // I now test that the safe distance between cars is respected and that I don't change
    // lane with not enough safe space
    // I slow down to car's speed to 0. It should become my speed target
    sensorFusionData.clear();
    leftCar[5] += 25;
    rightCar[5] -= 25;
    carAhead[3] = 0.0;

    sensorFusionData.push_back(leftCar);
    sensorFusionData.push_back(rightCar);
    sensorFusionData.push_back(carAhead);

    const BehaviorPlanner::HighLevelTrajectoryReport result2 = mBehaviorPlanner.computeNewTrajectory(Predictor::Warnings());

    EXPECT_EQ(result2.behavior, BehaviorPlanner::keepLane);
    EXPECT_FLOAT_EQ(result2.targetSpeedMs, x_dot);
    EXPECT_FLOAT_EQ(result2.targetSpeedMs, result2.recommendedTargetSpeed);
}

TEST_F(BehaviorPlannerTest, SlowDownToTakeThirdLane_prepareRightLaneChange)
{
    // TESTSTEP: Me(1st lane). Slow car ahead and another one slightly
    //           faster overtaking on my right (2nd lane).
    // EXPECTED: Slow down and wait to change lane twice to take empty
    //           third lane.

    /*

     |   | * |   |
     | * |   |   |
     |   | ^ |   |  <- The arrow shows faster speed
     |   | * |   |
     | $ |   |   |  <- I'm the $
     |   |   |   |
     |   | o |   |  <- Possibility to change lane on the o location
     |   |   |   |
     |   |   |   |
     |   | * |   |
     
     */

    std::vector<std::vector<double>> sensorFusionData;
    double id = 0.0;
    double x = 0.0;
    double y = 0.0;
    double slowest_speed = 15.0; // speeds are in mph
    double slow_speed = 20.0;
    double my_speed = 22.0;
    double y_dot = 0.0;
    double s = 20.0;

    double d = mSensorFusion.highway().getDFromLane<double>(firstLane);
    std::vector<double> leftCar = {id++,
                                   x, y,
                                   slowest_speed,
                                   y_dot,
                                   s + 30,
                                   d, firstLane};
    sensorFusionData.push_back(leftCar);

    mSensorFusion.updateMyAVData(x, y, s, d, 0.0, my_speed);

    ASSERT_EQ(mSensorFusion.myAV().lane, firstLane);

    d = mSensorFusion.highway().getDFromLane<double>(secondLane);
    std::vector<double> carFarAhead = {id++,x, y, slow_speed,
                                y_dot, s + 35, d, secondLane};

    std::vector<double> carAhead = {id++,x, y, slow_speed,
                                y_dot, s + 5, d, secondLane};
    sensorFusionData.push_back(carAhead);
    sensorFusionData.push_back(carFarAhead);

    mSensorFusion.updateCarsData(sensorFusionData);

    const BehaviorPlanner::HighLevelTrajectoryReport result = mBehaviorPlanner.computeNewTrajectory();

    EXPECT_EQ(result.behavior, BehaviorPlanner::prepareRightLaneChange);
    EXPECT_EQ(result.targetLane, thirdLane);
    // I expect to slow down to let the car in the second lane pass
    EXPECT_EQ(result.targetSpeedMs, policy::getSafePolicy(policy::maxSpeedMs));
    EXPECT_LT(result.recommendedTargetSpeed, slow_speed);

    
}

