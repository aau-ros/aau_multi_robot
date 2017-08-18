#ifndef BATTERY_STATE_UPDATER_INTERFACE_H
#define BATTERY_STATE_UPDATER_INTERFACE_H

#include <ros/ros.h>
#include <explorer/battery_state.h>
#include <explorer/Speed.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <robot_state/robot_state_management.h>
#include <utilities/time_manager.h>
#include "robot_state_manager_interface.h"

//class BatteryStateUpdater : public RobotStateHandler
class BatteryStateUpdaterInterface
{
public:
    virtual void setTimeManager(TimeManagerInterface *time_manager) = 0;
    virtual void setRobotStateManager(RobotStateManagerInterface *robot_state_manager) = 0;
    virtual void logMetadata() = 0;
    virtual void createLogDirectory() = 0;
    virtual void updateBatteryState() = 0;
//    void handle(InitializingState *state) override;
//    void handle(ChoosingActionState *state) override;
//    void handle(ComputingNextGoalState *state) override;
};

#endif // BATTERY_STATE_UPDATER_INTERFACE_H
