#include <iostream>
#include <vector>
#include "gtest/gtest.h"

#include "drivingpolicy.h"
#include "roadtypes.h"

class HighwayTests : public ::testing::Test
{
public:

    HighwayTests()
    : mHighway()
    {}

    virtual void SetUp();

    virtual void TearDown() {
    }

    Highway mHighway; ///< Representation of the highway I'm driving on

};

void HighwayTests::SetUp()
{
    const int numberLanes = 2;
    mHighway.setNumberLanes(numberLanes);
}

TEST_F(HighwayTests, GetSetNumberLanes)
{
    const int numberLanes = 2;
    mHighway.setNumberLanes(numberLanes);

    ASSERT_EQ(mHighway.getNumberLanes(), numberLanes);

    ASSERT_DEATH(mHighway.setNumberLanes(0), "");
}

TEST_F(HighwayTests, OngoingLaneChange)
{
    // TESTSTEP: I give my d position as being the one of the first lane
    // EXPECTED: I expect NOT to be considered changing lane.
    ASSERT_FALSE(mHighway.ongoingLaneChange(mHighway.getDFromLane<double>(firstLane)));

    // TESTSTEP: I give my d position as being in-between the first and second lane
    // EXPECTED: I expect to be considered changing lane.
    const double inbetweenFirstAndSecondLanes = (mHighway.getDFromLane<double>(firstLane) +
                                                 mHighway.getDFromLane<double>(secondLane)) / 2.0;
    ASSERT_TRUE(mHighway.ongoingLaneChange(inbetweenFirstAndSecondLanes));

    // TESTSTEP: I give my d position as being less than 1 meter away from the center of the first lane
    // EXPECTED: I expect NOT to be considered changing lane.
    const double deviation = 0.99;
    ASSERT_FALSE(mHighway.ongoingLaneChange(mHighway.getDFromLane<double>(firstLane) + deviation));
    ASSERT_FALSE(mHighway.ongoingLaneChange(mHighway.getDFromLane<double>(firstLane) - deviation));

    // TESTSTEP: I give my d position as being exactly 1 meter away from the center of the first lane
    // EXPECTED: I expect to be considered changing lane.
    const double largerDeviation = 1.0;
    ASSERT_TRUE(mHighway.ongoingLaneChange(mHighway.getDFromLane<double>(firstLane) + largerDeviation));
    ASSERT_TRUE(mHighway.ongoingLaneChange(mHighway.getDFromLane<double>(firstLane) - largerDeviation));
}
