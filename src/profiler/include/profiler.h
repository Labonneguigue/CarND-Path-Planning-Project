#ifndef PROFILER_H
#define PROFILER_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>

#include "entry.h" // Entry

class Profiler
{
public:

    Profiler(std::ostream& objOstream);

    ~Profiler();

    void start(const std::string name);

    void start(const std::string name, const double callsBeforePrint);

    void stop(const std::string name);

    void setPrintFrequency(const std::string name, const double callsBeforePrint);

    void print();

private:

    std::vector<Entry> mEntries;

    std::string mPrintTrigger;
    double mCallsBeforePrint;

    std::ostream& mOstream; ///< Output stream instance. Can be file, std::cout or other
};

#endif //PROFILER_H
