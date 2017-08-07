#ifndef ROBOT_STATE_EM_H
#define ROBOT_STATE_EM_H

class Computer;

class RobotStateEM {
public:
    void updateBatteryState(Computer *c) {};
};

class RobotState1 : public RobotStateEM
{
public:
    RobotState1();
//    void updateBatteryState(Computer *c) override {};
};

class RobotState2: public RobotStateEM
{
public:
    RobotState2();
//    void updateBatteryState(Computer *c) override {};
};

class RobotState3 : public RobotStateEM
{
public:
    RobotState3();
//    void updateBatteryState(Computer *c) override {};
};

#endif // ROBOT_STATE_H
