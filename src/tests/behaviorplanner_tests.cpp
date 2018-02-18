
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
    std::vector<std::vector<double>> sensorFusionData;
    double id = 0;
    double x = 0;
    double y = 0;
    double x_dot = 0;
    double y_dot = 0;
    double s = 20;
    Lane lane = firstLane;
    double d = Highway::getDFromLane<double>(lane);
    std::vector<double> car = {id,x, y, x_dot, y_dot, s, d, lane};

    sensorFusionData.push_back(car);
    mSensorFusion.updateCarsData(sensorFusionData);

    mSensorFusion.updateMyAVData(x, y, 0, d, 0.0, 20);


}

TEST_F(BehaviorPlannerTest, SlowDownToFullStop)
{
    const BehaviorPlanner::HighLevelTrajectoryReport result = mBehaviorPlanner.computeNewTrajectory(Predictor::Warnings());

    ASSERT_EQ(result.behavior, BehaviorPlanner::rightLaneChange);
    ASSERT_FALSE(result.warningTriggered);
}
