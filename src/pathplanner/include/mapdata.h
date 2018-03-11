#ifndef MAP_DATA_H
#define MAP_DATA_H

#include <vector>
#include <cassert>

struct MapData
{
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;

    /**
     *
     */
    MapData()
    {}

    /**
     *
     */
    bool initialize(std::vector<double>& s,
                    std::vector<double>& x,
                    std::vector<double>& y)
    {
        waypoints_s = s;
        waypoints_x = x;
        waypoints_y = y;

        if ( (waypoints_s.size() > 0)
            && (waypoints_x.size() > 0)
            && (waypoints_y.size() > 0) )
        {
            return true;
        }
        return false;
    }
};

#endif // MAP_DATA_H
