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
     * @param[in] nbLanes Number of lanes on the current highway.
     *
     * @note Each lanes has an incrementing number starting a 0 for
     *       the leftmost lane (Assume right lane driving).
     *       Default number of lanes is 0.
     */
    Highway(const int nbLanes = 1)
    : mLanes()
    , mLaneWidth(4.0)
    {
        setNumberLanes(nbLanes);
    }

    /** Returns the number of lanes on the road
     *
     * @return Number of lanes on the road
     */
    int getNumberLanes() const
    {
        return mLanes.size();
    }

    /** Returns the lane number of each lane on this road
     *
     * @return vector<int> lane numbers to each available lane
     */
    const std::vector<Lane> getAvailableLanes() const
    {
        return mLanes;
    }

    /** Return true if the car undertaking a lane change,
     *  false otherwise
     *
     * @param[in] vehicle_d D coordinate of the car
     *
     */
    inline bool ongoingLaneChange(double vehicle_d) const
    {
        const double laneChangeOverThreshold = mLaneWidth * 0.25;
        for (unsigned int lane = 0 ; lane < mLanes.size() ; ++lane)
        {
            if (isCarInLane(mLanes[lane], vehicle_d, laneChangeOverThreshold)) return false;
        }
        return true;
    }

    /** Changes dynamically the size of the road to adapt to another
     *  size of highway
     *
     * @param[in] nbLanes Number of lanes that contains this highway
     *
     */
    void setNumberLanes(const int nbLanes)
    {
        mLanes.clear();

        for (int lane = 0; lane < nbLanes ; ++lane)
        {
            mLanes.push_back(static_cast<Lane>(lane));
        }

        assert(mLanes.size() > 0);
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
        assert(static_cast<unsigned int>(currentLane) < mLanes.size());
        assert(currentLane >= 0);

        std::vector<Lane> deltaLanes;
        for (unsigned int lane = 0; lane < mLanes.size() ; ++lane)
        {
            deltaLanes.push_back(static_cast<Lane>(mLanes[lane]-currentLane));
        }
        return deltaLanes;
    }

    /**
     Returns the d component in the Frenet coordinates space for a particular lane

     @param double laneNumber Lane number. 0 is left, 1 is center and 2 is right lane
     @return d Frenet component
     */
    template <typename T>
    T getDFromLane(Lane laneNumber) const
    {
        return ((mLaneWidth/2.0) + (mLaneWidth * static_cast<T>(laneNumber)));
    }

    /**
     Checks whether a car is a specified lane or not

     @param lane Enum of the lane I'm in: 0 is left, 1 is center and 2 is right lane
     @param d Frenet d coordinate of a car
     @return True if the car is in my lane, False otherwise

     @tparam T type of the othercar_d frenet coordinate
     */
    bool isCarInLane(const Lane lane, const double d, double window = -1) const
    {
        if (window == -1) window = (mLaneWidth / 2.0);
        return ((d < (getDFromLane<double>(lane) + window)) &&
                (d > (getDFromLane<double>(lane) - window))) ? true : false;
    }

    std::vector<Lane> mLanes; ///< vector representing the lanes. Each integer is the lane number
    double mLaneWidth; ///< Lane width in meters - Can be changed dynamically when the car changes road

    constexpr const static Lane mInitialLane = secondLane; ///< Hardcoded initial lane for initialisation

};


#endif //ROAD_TYPES_H
