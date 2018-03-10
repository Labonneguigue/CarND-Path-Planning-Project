#ifndef DRIVING_POLICY_H
#define DRIVING_POLICY_H

#include "utl.h"

namespace policy
{
    constexpr static const bool keepRightLane = false; ///< False: US, True: EU

    constexpr static const int defaultNbLanes = 3; ///< Default number of lanes on the road at startup time
    constexpr static const int safeDistance = 10; ///< Distance to always keep between cars
    constexpr static const double detectionDistance = 35.0; ///< Distance at which behavior needs to start be modified
    constexpr static const double maxSpeedMph = 50.0; ///< Maximum allowed speed in mph
    constexpr static const double maxSpeedMs = utl::mph2ms(maxSpeedMph); ///< Maximum allowed speed in m/s
    constexpr static const double maxAccelerationMs = 10.0; ///< Maximum allowed acceleration in m/s^2
    constexpr static const double maxJerkMs = 50.0; ///< Maximum allowed jerk (time derivative of the acceleration) in m/s^3

    constexpr static const double safeMaxPolicyPercent = 0.97; ///< Returns percentage of the absolute limit that can be used to
    constexpr static const double simulatorDeltaT = 0.02; ///< Each trajectory waypoints are met every 0.02seconds in the simulator

    template <typename T>
    constexpr int getSafePolicy(const T policy)
    {
        return safeMaxPolicyPercent * policy;
    }
};

#endif // DRIVING_POLICY_H
