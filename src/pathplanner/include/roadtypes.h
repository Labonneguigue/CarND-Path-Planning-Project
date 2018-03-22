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
     * @param[in] nbLanes Number of lanes on the current highway
     *
     * @note Each lanes has an incrementing number starting a 0 for
     *       the leftmost lane (Assume right lane driving)
     */
    Highway(const int nbLanes)
    : lanes()
    {
        setNumberLanes(nbLanes);
    }

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
     * @return vector<int> lane numbers to each available lane
     */
    const std::vector<Lane> getAvailableLanes() const
    {
        return lanes;
    }

    /** Changes dynamically the size of the road to adapt to another
     *  size of highway
     *
     * @param[in] nbLanes Number of lanes that contains this highway
     *
     */
    void setNumberLanes(const int nbLanes)
    {
        lanes.clear();

        for (int lane = 0; lane < nbLanes ; ++lane)
        {
            lanes.push_back(static_cast<Lane>(lane));
        }

        assert(lanes.size() > 0);
    }

    /** Returns delta in number of lane to each available lanes on this road
     *
     * @param[in] currentLane Lane myAV is currently on
     *
     * @return vector<int> difference in lane numbering to each available lane
     */
    const std::vector<Lane> getDeltaToAvailableLanes(const Lane currentLane) const
    {
        // Can't be in another lane than the one represented by this struct
        assert(currentLane < lanes.size());
        assert(currentLane >= 0);

        std::vector<Lane> deltaLanes;
        for (int lane = 0; lane < lanes.size() ; ++lane)
        {
            deltaLanes.push_back(static_cast<Lane>(lanes[lane]-currentLane));
        }
        return deltaLanes;
    }

    /**
     Returns the d component in the Frenet coordinates space for a particular lane

     @param double laneNumber Lane number. 0 is left, 1 is center and 2 is right lane
     @return d Frenet component
     */
    template <typename T>
    static T getDFromLane(Lane laneNumber)
    {
        static const T laneWidth = 4;
        static const T laneBias = 2;
        return (laneBias + (laneWidth * laneNumber));
    }

    /**
     Checks whether a car is a specified lane or not

     @param lane Enum of the lane I'm in: 0 is left, 1 is center and 2 is right lane
     @param d Frenet d coordinate of a car
     @return True if the car is in my lane, False otherwise

     @tparam T type of the othercar_d frenet coordinate
     */
    template <typename T>
    static bool isCarInLane(Lane lane, T d)
    {
        static const int halfLaneWidth = 2;
        return ((d < (getDFromLane<T>(lane) + halfLaneWidth)) &&
                (d > (getDFromLane<T>(lane) - halfLaneWidth))) ? true : false;
    }

    std::vector<Lane> lanes; ///< vector representing the lanes. Each integer is the lane number

    constexpr const static Lane initialLane = secondLane; ///< Hardcoded initial lane for initialisation
};


#endif //ROAD_TYPES_H
