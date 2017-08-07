#ifndef ROBOT_STATE_EM_H
#define ROBOT_STATE_EM_H

#include "computer.h"

class RobotStateEM {
public:
    virtual void accept(Computer *c) = 0;
};

class RobotState1 : public RobotStateEM
{
public:
    RobotState1();
    void accept(Computer *c) override {c->execute(this);};
};

class RobotState2: public RobotStateEM
{
public:
    RobotState2();
    void accept(Computer *c) override {c->execute(this);};
};

class RobotState3 : public RobotStateEM
{
public:
    RobotState3();
    void accept(Computer *c) override {c->execute(this);};
};

#endif // ROBOT_STATE_H
