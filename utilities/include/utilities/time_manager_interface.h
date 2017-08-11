#ifndef TIME_MANAGER_INTERFACE_H
#define TIME_MANAGER_INTERFACE_H

#include <ros/ros.h>

class TimeManagerInterface
{
public:

    virtual ros::Time simulationTimeNow() = 0;

};

#endif /* TIME_MANAGER_INTERFACE_H */
