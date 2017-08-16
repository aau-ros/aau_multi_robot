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
        INITIALIZING,
        CHOOSING_ACTION,
        COMPUTING_NEXT_GOAL,
        MOVING_TO_FRONTIER,
        GOING_CHECKING_VACANCY,
        CHECKING_VACANCY,
        GOING_CHARGING,
        CHARGING,
        CHARGING_COMPLETED,
        CHARGING_ABORTED,
        LEAVING_DS,
        GOING_IN_QUEUE,
        IN_QUEUE,
        AUCTIONING,

        // MISSING
        auctioning_2,
        exploring_for_graph_navigation,
        choosing_next_action,
        stopped,
        dead,
        stuck,
        auctioning_3,
        moving_to_frontier_before_going_charging,
        finished,
        moving_away_from_ds,
        fully_charged,
        leaving_ds
    };
}

//TODO put this in the namespace?
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
