#ifndef COMPUTER_H
#define COMPUTER_H

#include <ros/ros.h>
#include <explorer/battery_state.h>

class RobotState1;
class RobotState2;
class RobotState3;

class Computer //TODO do we need Computer and Computer2?
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
    Computer2(explorer::battery_state *b);
    void execute(RobotState1 *r) override {};
    void execute(RobotState2 *r) override {};
    void execute(RobotState3 *r) override {};

private:
    double speed_avg_init; //TODO speed_avg maybe is not a good name
    double power_charging;              // W (i.e, watt)
    double power_per_speed;             // W/(m/s)
    double power_moving_fixed_cost;     // W/(m/s)
    double power_sonar;                 // W
    double power_laser;                 // W
    double power_microcontroller;       // W
    double power_basic_computations;    // W
    double power_advanced_computations; // W
    double max_speed_linear;            // m/s
    double maximum_traveling_distance;  // m/s
    explorer::battery_state *b;
};

#endif // COMPUTER_H
