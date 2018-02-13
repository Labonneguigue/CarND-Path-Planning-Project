#ifndef ROAD_TYPES_H
#define ROAD_TYPES_H

#include <cassert>
#include <vector>


/** Lane enumerator that represent the different lanes
 *  on the road
 *
 * @note The lanes are numbered from left to right
 */
enum Lane
{
    undefined = -1,
    firstLane = 0,
    secondLane = 1,
    thirdLane = 2
};

/** Highway structure represents a highway and provide
 *  convenient way to obtain information about the other
 *  available lanes given the current lane.
 *  The way Highway is built allows the car to change road
 *  and dynamically change the number of lanes available on
 *  the road
 */
struct Highway
{

    /** Constructor
     *
     * @param nbLanes
     *
     * @note Each lanes has an incrementing number starting a 0 for
     *       the leftmost lane (Assume right lane driving)
     */
    Highway(const int nbLanes)
    : lanes(nbLanes)
    {}

    /** Returns the number of lanes on the road
     *
     * @return Number of lanes on the road
     */
    const int getNumberLanes() const
    {
        return lanes.size();
    }

    /** Returns the lane number of each lane on this road
     *
     * @param[in] currentLane Lane myAV is currently on
     *
     * @return vector<int> lane numbers to each available lane
     */
    const std::vector<int> getAvailableLanes(const int currentLane) const
    {
        // Can't be in another lane than the one represented by this struct
        assert(currentLane < lanes.size());
        assert(currentLane >= 0);

        std::vector<int> availableLanes;
        for (int lane = 0; lane < lanes.size() ; ++lane)
        {
            availableLanes.push_back(lane);
        }
        return availableLanes;
    }

    /** Returns delta in number of lane to each available lanes on this road
     *
     * @param[in] currentLane Lane myAV is currently on
     *
     * @return vector<int> difference in lane numbering to each available lane
     */
    const std::vector<int> getDeltaToAvailableLanes(const int currentLane) const
    {
        // Can't be in another lane than the one represented by this struct
        assert(currentLane < lanes.size());
        assert(currentLane >= 0);

        std::vector<int> availableLanes;
        for (int lane = 0; lane < lanes.size() ; ++lane)
        {
            availableLanes.push_back(lane-currentLane);
        }
        return availableLanes;
    }

    std::vector<int> lanes; ///< vector representing the lanes. Each integer is the lane number
};


#endif //ROAD_TYPES_H
