#ifndef MAP_DATA_H
#define MAP_DATA_H

#include <vector>
#include <cassert>

enum RoadLanes
{
    leftLane = 0,
    centerLane = 1,
    rightLane = 2
};

struct MapData
{
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;

    MapData(std::vector<double>& s,
            std::vector<double>& x,
            std::vector<double>& y)
    : waypoints_s(s)
    , waypoints_x(x)
    , waypoints_y(y)
    {
        assert(waypoints_s.size() > 0);
        assert(waypoints_x.size() > 0);
        assert(waypoints_y.size() > 0);
    }
};

#endif // MAP_DATA_H
