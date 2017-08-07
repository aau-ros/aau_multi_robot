#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <iostream>

namespace robot_state
{
    //TODO complete
    //TODO complete description
    //TODO add classes for Visitor
    enum robot_state_t
    {
        INITIALIZING,
        CHOOSING_ACTION,
        COMPUTING_NEXT_GOAL,              // the robot is computing which is the next goal (frontier, docking station, ...)
        MOVING_TO_FRONTIER,              // the robot is moving to a selected frontier
        GOING_CHECKING_VACANCY, // the robot is approaching a DS to check if it is actually free
        CHECKING_VACANCY,
        GOING_CHARGING,
        CHARGING,
        CHARGING_COMPLETED,
        CHARGING_ABORTED,
        LEAVING_DS,
        GOING_IN_QUEUE,
        IN_QUEUE,
        AUCTIONING
    };

}

#endif // ROBOT_STATE_H
