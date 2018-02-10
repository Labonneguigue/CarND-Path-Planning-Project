#ifndef CONTROLLER_FEEDBACK_H
#define CONTROLLER_FEEDBACK_H

#include <vector>

struct ControllerFeedback
{
    std::vector<double> remainingPath_x;
    std::vector<double> remainingPath_y;

    ControllerFeedback(std::vector<double>& x,
                       std::vector<double>& y)
    : remainingPath_x(x)
    , remainingPath_y(y)
    {}

};

#endif // CONTROLLER_FEEDBACK_H
