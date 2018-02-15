#include <fstream>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include "gtest/gtest.h"
#include "profiler.h"

#define MAX_ABSOLUTE_ERROR 1e-4

class ProfilerTest : public ::testing::Test
{
public:

    ProfilerTest()
    {}

    virtual void SetUp() {

    }

    virtual void TearDown() {

    }

};


TEST_F(ProfilerTest, LoggingToFile) {

    // Test verifies that the profiler can log data to file

    std::string testName = "Test";
    std::string fileName = "../../../../src/tests/testprofiler.cpp";

    // First check file empty

    // Then write to it using the profiler
    std::ofstream profilerOutputFile;
    profilerOutputFile.open(fileName);
    
    Profiler profiler(profilerOutputFile);
    profiler.start(testName);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    profiler.stop(testName);
    profiler.print();

    // Check that something was printed

}
