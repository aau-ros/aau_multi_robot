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
        INITIALIZING,           // the robot has just been activated
        CHOOSING_ACTION,        // the robot is checking if it won an auction and so it has to recharge or if it can try to check if it can reach a new frontier
        COMPUTING_NEXT_GOAL,    // the robot is computing which is the next goal (frontier, docking station, ...)
        MOVING_TO_FRONTIER,     // the robot is moving to a selected frontier
        GOING_CHECKING_VACANCY, // the robot is approaching the optimal DS to check if it is actually free
        CHECKING_VACANCY,       // the robot is in proximity of the optimal DS and it's checking if it's actually free
        GOING_CHARGING,         // the robot has checked that the optimal DS is actually free, and so it is going to occupy it to recharge
        CHARGING,               // the robot is charging at the optimal DS
        CHARGING_COMPLETED,     // the robot has just finished charging
        CHARGING_ABORTED,       // the charging has been aborted due to action lost
        LEAVING_DS,             // the robot is leaving the optimal DS
        GOING_IN_QUEUE,         // the robot has lost an auction started by itself, and so it has to queue
        IN_QUEUE,               // the robot is in queue, waiting for the optimal DS to because vacant
        AUCTIONING              // the robot has just started its own auction
    };

}

#endif // ROBOT_STATE_H
