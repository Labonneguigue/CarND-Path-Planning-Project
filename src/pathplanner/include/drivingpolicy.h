#ifndef DRIVING_POLICY_H
#define DRIVING_POLICY_H


namespace policy
{
    constexpr static const bool keepRightLane = false; ///< False: US, True: EU

    constexpr static const int defaultNbLanes = 3; ///< Default number of lanes on the road at startup time
    constexpr static const int safeDistance = 25; ///< Distance to always keep between cars
    constexpr static const int detectionDistance = 40; ///< Distance at which behavior needs to start be modified
    constexpr static const int maxAccelerationMs = 10; ///< Maximum allowed acceleration in m/s
    constexpr static const int maxSpeedMph = 50; ///< Maximum allowed speed in mph

    constexpr static const double safeMaxPolicyPercent = 0.95; ///< Returns percentage of the absolute limit that can be used to

    template <typename T>
    constexpr int getSafePolicy(const T policy)
    {
        return safeMaxPolicyPercent * policy;
    }
};

#endif // DRIVING_POLICY_H
