#include <cassert>
#include <iomanip>

#include "entry.h"

Entry::Entry(std::string name)
: mName(name)
, mStartTime(std::chrono::high_resolution_clock::time_point::min())
, mEndTime(std::chrono::high_resolution_clock::time_point::min())
, mCounting(false)
, mMinimumRuntime(std::numeric_limits<double>::max())
, mMaximumRuntime(0)
, mNumberCalls(0)
, mCumulativeRuntime(0)
, mLastRuntime(0)
{}

Entry::~Entry()
{}

void Entry::start(){
    mCounting = true;
    mStartTime = std::chrono::high_resolution_clock::now();
}

void Entry::stop()
{
    mEndTime = std::chrono::high_resolution_clock::now();
    // Assert that the start method was called
    assert(mCounting == true);
    mCounting = false;
    ++mNumberCalls;
    mLastRuntime = std::chrono::duration_cast<std::chrono::microseconds>(mEndTime - mStartTime).count();
    mCumulativeRuntime += mLastRuntime;
    if (mLastRuntime < mMinimumRuntime)
    {
        mMinimumRuntime = mLastRuntime;
    }
    if (mLastRuntime > mMaximumRuntime)
    {
        mMaximumRuntime = mLastRuntime;
    }
}

const std::string Entry::getName()
{
    return mName;
}

const double Entry::getCount()
{
    return mNumberCalls;
}

void Entry::print(std::ostream& outputStream)
{
    const std::string separator = "  |  ";
    outputStream << std::setw(15) << mName << separator
                 << std::setw(15) << mNumberCalls << separator
                 << std::setw(15) << mCumulativeRuntime / 1000 << separator
                 << std::setw(15) << mMinimumRuntime << separator
                 << std::setw(15) << mMaximumRuntime << separator
                 << std::setw(15) << mCumulativeRuntime / mNumberCalls << separator
                 << std::endl;
}

void Entry::printHeader(std::ostream &outputStream)
{
    const std::string separator = "  |  ";
    outputStream << std::setw(15) << "Call name" << separator
                 << std::setw(15) << "Number of calls" << separator
                 << std::setw(15) << "Cum runtime(ms)" << separator
                 << std::setw(15) << "Min runtime(us)" << separator
                 << std::setw(15) << "Max runtime(us)" << separator
                 << std::setw(15) << "Avg runtime(us)" << separator
                 << std::endl;
}
