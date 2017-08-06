#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <iostream>

namespace robot_state
{
    //TODO complete
    //TODO complete description
    enum robot_state_enum
    {
        COMPUTING,              // the robot is computing which is the next goal (frontier, docking station, ...)
        EXPLORING,              // the robot is moving to a selected frontier
        GOING_CHECKING_VACANCY, // the robot is approaching a DS to check if it is actually free
        CHECKING_VACANCY,
        GOING_CHARGING,
        CHARGING,
        FULLY_CHARGED,
        LEAVING_DS,
        GOING_IN_QUEUE,
        IN_QUEUE,
        AUCTIONING
    };

}

#endif // ROBOT_STATE_H
