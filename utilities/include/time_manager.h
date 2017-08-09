#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

#include <ros/ros.h>
#include "time_manager_interface.h"

class TimeManager : public TimeManagerInterface
{
public:
    /**
     * Constructor.
     */
    TimeManager();

    ros::Time simulationTimeNow();

};

#endif /* TIME_MANAGER_H */
