#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

namespace robot_state
{
    enum robot_state_enum
    {
        COMPUTING,              // the robot is computing which is the next goal (frontier, docking station, ...)
        EXPLORING,              // the robot is moving to a selected frontier
        GOING_CHECKING_VACANCY, // the robot is approaching a DS to check if it is actually free
        CHECKING_VACANCY,
        GOING_CHARGING,
        CHARGING,
        //FULLY_CHARGED,
        LEAVING_DS,
        GOING_IN_QUEUE,
        IN_QUEUE
    };
}

#endif // ROBOT_STATE_H
