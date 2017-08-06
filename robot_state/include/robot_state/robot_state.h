#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

namespace robot_state
{
    static const unsigned int EXPLORING = 0;
    enum robot_state
    {
        COMPUTING // the robot is computing which is the next goal (frontier, docking station, ...)
    };
}

#endif // ROBOT_STATE_H
