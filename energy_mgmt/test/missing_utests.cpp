#include <ros/ros.h>
#include <gtest/gtest.h>
#include <explorer/battery_state.h>
#include "robot_state_manager.h"
#include "battery_simulate.h"
#include "battery_state_updater.h"
#include "docking.h"

TEST(TestAuctionManager, testMissing)
{
    RobotStateManager rsm("test");
    battery_simulate bs;
    explorer::battery_state state;
    BatteryStateUpdater bsu(&state);
    docking d;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "missing_utests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
