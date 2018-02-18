
#include <iostream>
#include <vector>

#include "roadtypes.h"
#include "utl.h"
#include "gtest/gtest.h"

TEST(UTL_H_TESTS, isCarInLane_FirstLane)
{
    const Lane lane = firstLane;
    const int d = 0;
    ASSERT_EQ(lane, Highway::isCarInLane(lane, d));

}

TEST(UTL_H_TESTS, isCarInLane_Undefined)
{
    const Lane lane = undefined;
    const int d = 4;
    ASSERT_EQ(lane, Highway::isCarInLane(lane, d));
}
