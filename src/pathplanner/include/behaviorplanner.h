#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include "sensorfusion.h"
#include "vehicledata.h"

class BehaviorPlanner
{
public:

    /** Default constructor
     *
     */
    BehaviorPlanner(SensorFusion& sensorFusion);

    /** Default destructor
     *
     */
    ~BehaviorPlanner();

    /**
     *
     */
    void updateState(VehicleData& vehicle);

private:
    
    SensorFusion& mSensorFusion;
    
};

#endif //BEHAVIOR_PLANNER_H
