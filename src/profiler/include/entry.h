#ifndef ENTRY_H
#define ENTRY_H

#include <string>
#include <chrono>
#include <iostream>

class Entry
{
public:

    Entry(std::string name);

    ~Entry();

    void start();

    void stop();

    const std::string getName();

    const double getCount();

    void print(std::ostream& outputStream);

    static void printHeader(std::ostream& outputStream);

private:

    std::string mName;

    std::chrono::high_resolution_clock::time_point mStartTime;
    std::chrono::high_resolution_clock::time_point mEndTime;

    bool mCounting;
    double mMinimumRuntime;
    double mMaximumRuntime;
    double mNumberCalls;
    double mCumulativeRuntime;
    double mLastRuntime;

};

#endif //ENTRY_H
