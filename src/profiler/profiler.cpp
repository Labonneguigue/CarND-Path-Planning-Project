#include <cmath>
#include <cassert>
#include "profiler.h"

Profiler::Profiler(std::ostream& objOstream)
: mOstream(objOstream)
{}

Profiler::~Profiler()
{
    std::flush(mOstream);
}

void Profiler::start(const std::string name)
{
    bool entryExist = false;
    for (unsigned int entry = 0; entry < mEntries.size(); ++entry)
    {
        if ( name == mEntries[entry].getName() )
        {
            mEntries[entry].start();
            entryExist = true;
        }
    }
    if (!entryExist)
    {
        // If entry doesn't exist I create it and append it to my vector of entries
        mEntries.emplace_back(Entry(name));
        assert(mEntries[mEntries.size()-1].getName() == name);
        mEntries[mEntries.size()-1].start();
    }
}

void Profiler::start(const std::string name, const double callsBeforePrint)
{
    // Set print frequency
    mPrintTrigger = name;
    mCallsBeforePrint = callsBeforePrint;
    start(name);
}

void Profiler::stop(const std::string name)
{
    bool entryExist = false;
    for (unsigned int entry = 0; entry < mEntries.size(); ++entry)
    {
        if ( name == mEntries[entry].getName() )
        {
            mEntries[entry].stop();
            entryExist = true;
            if (name == mPrintTrigger && (std::fmod(mEntries[entry].getCount(),mCallsBeforePrint) == 0))
            {
                print();
            }
        }
    }
    // Assert that the entry exist 
    assert(entryExist);
}

void Profiler::print()
{
    Entry::printHeader(mOstream);
    for (unsigned int entry = 0; entry < mEntries.size(); ++entry)
    {
        mEntries[entry].print(mOstream);
    }
    mOstream << "\n" << std::endl;
    std::flush(mOstream);
}

