#ifndef COMPUTER_H
#define COMPUTER_H

#include <explorer/battery_state.h>

class RobotState1;
class RobotState2;
class RobotState3;

class Computer
{
public:
    Computer() {};
    virtual void execute(RobotState1 *r) = 0;
    virtual void execute(RobotState2 *r) = 0;
    virtual void execute(RobotState3 *r) = 0;
};

class Computer2 : public Computer
{
public:
    Computer2(explorer::battery_state *b) {this->b = b;};
    void execute(RobotState1 *r) override {};
    void execute(RobotState2 *r) override {};
    void execute(RobotState3 *r) override {};

private:
    explorer::battery_state *b;
};

#endif // COMPUTER_H
