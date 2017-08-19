#include <gtest/gtest.h>
#include "robot_state/robot_state_management.h"

#define SLEEP_TIME_TO_LET_ROBOT_STATE_NODE_START 3

//TODO remove dublicated code (for instance for declare the services)
TEST(RobotStatePkgApiTest, testGetRobotStateEnum)
{
    RobotStateApi manager;

//    manager.getRobotState();
//    bool succees = (InitializingState * s = dynamic_cast<InitializingState*>(manager.getRobotState())); //TODO
//    EXPECT_TRUE(succees);

    EXPECT_EQ(robot_state::INITIALIZING, manager.getRobotStateEnum());
    EXPECT_NE(robot_state::IN_QUEUE, manager.getRobotStateEnum());    
}

TEST(RobotStatePkgApiTest, testSetRobotStateEnum)
{
    RobotStateApi manager;
    manager.setRobotStateEnum(robot_state::IN_QUEUE);
    EXPECT_NE(robot_state::INITIALIZING, manager.getRobotStateEnum());
    EXPECT_EQ(robot_state::IN_QUEUE, manager.getRobotStateEnum());    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "robot_state");

    // Wait some seconds to let robot_state node start
    ros::start(); // required to use ros::Time and ros::Duration
    ros::Duration(SLEEP_TIME_TO_LET_ROBOT_STATE_NODE_START).sleep();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
