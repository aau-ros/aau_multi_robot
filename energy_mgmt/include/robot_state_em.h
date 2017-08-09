#ifndef ROBOT_STATE_EM_H
#define ROBOT_STATE_EM_H

#include "computer.h"

class RobotStateEM {
public:
    virtual void accept(Computer *c) = 0;
};

class InitializingState : public RobotStateEM
{
public:
    InitializingState() {};
    void accept(Computer *c) override {c->execute(this);};
};


#endif // ROBOT_STATE_H
