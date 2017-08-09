#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <iostream>
#include "robot_state_handler.h"

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



class InitializingState;
class ChoosingActionState;
class ComputingNextGoalState;
class MovingToFrontierState;
class GoingCheckingVacancyState;
class CheckingVanancyState;
class GoingChargingState;
class ChargingStateState;
class ChargingCompletedState;
class ChargingAbortedState;
class LeavingDsState;
class GoingInQueueState;
class InQueueState;
class AuctioningState;

//TODO move in separate file
class RobotStateHandler //TODO do we need Computer and Computer2? //TODO RobotStateVisitor
{
public:
    RobotStateHandler() {};
    virtual void handle(InitializingState *state) = 0;
    virtual void handle(ChoosingActionState *state) = 0;
    virtual void handle(ComputingNextGoalState *state) = 0;
//    virtual void handle(MovingToFrontierState *state) = 0;
//    virtual void handle(GoingCheckingVacancyState *state) = 0;
//    virtual void handle(CheckingVanancyState *state) = 0;
//    virtual void handle(ChargingStateState *state) = 0;
//    virtual void handle(ChargingCompletedState *state) = 0;
//    virtual void handle(ChargingAbortedState *state) = 0;
//    virtual void handle(LeavingDsState *state) = 0;
//    virtual void handle(GoingInQueueState *state) = 0;
//    virtual void handle(InQueueState *state) = 0;
//    virtual void handle(AuctioningState *state) = 0;
};









class RobotState {
public:
    virtual void accept(RobotStateHandler *handler) = 0;
};

class InitializingState : public RobotState
{
public:
    InitializingState() {};
    void accept(RobotStateHandler *handler) override {handler->handle(this);};
};

class ChoosingActionState : public RobotState
{
public:
    ChoosingActionState() {};
    void accept(RobotStateHandler *handler) override {handler->handle(this);};
};

//class ComputingNextGoalState : public RobotState
//{
//public:
//    ComputingNextGoalState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class MovingToFrontierState : public RobotState
//{
//public:
//    MovingToFrontierState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class GoingCheckingVacancyState : public RobotState
//{
//public:
//    GoingCheckingVacancyState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class CheckingVanancyState : public RobotState
//{
//public:
//    CheckingVanancyState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class GoingChargingState : public RobotState
//{
//public:
//    GoingChargingState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class ChargingStateState : public RobotState
//{
//public:
//    ChargingState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class ChargingCompletedState : public RobotState
//{
//public:
//    ChargingCompletedState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class ChargingAbortedState : public RobotState
//{
//public:
//    ChargingAbortedState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class LeavingDsState : public RobotState
//{
//public:
//    LeavingDsState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class GoingInQueueState : public RobotState
//{
//public:
//    GoingInQueueState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class InQueueState : public RobotState
//{
//public:
//    InQueueState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};

//class AuctioningState : public RobotState
//{
//public:
//    AuctioningState() {};
//    void accept(RobotStateHandler *handler) override {handler->handle(this);};
//};


#endif // ROBOT_STATE_H
