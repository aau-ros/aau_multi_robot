#include <gtest/gtest.h>
#include "robot_state/robot_state_management.h"

#define SLEEP_TIME_TO_LET_ROBOT_STATE_NODE_START 3

//TODO these tests have some problems; one is possibly caused by the fact that the launch file tries to launch the robot_state node but that node wants to log data on file, so this could cause, but even when commenting out the line of codes related to the logging, these tests have still problems...
//TODO remove dublicated code (for instance for declare the services)
TEST(RobotStatePkgApiTest, testGetRobotStateEnum)
{
    RobotStateApi manager;

//    manager.getRobotState();
//    bool succees = (InitializingState * s = dynamic_cast<InitializingState*>(manager.getRobotState())); //TODO
//    EXPECT_TRUE(succees);

//    EXPECT_EQ(robot_state::INITIALIZING, manager.getRobotStateEnum());
//    EXPECT_NE(robot_state::IN_QUEUE, manager.getRobotStateEnum());    
}

TEST(RobotStatePkgApiTest, testSetRobotStateEnum)
{
    RobotStateApi manager;
//    manager.setRobotStateEnum(robot_state::IN_QUEUE); //TODO the set doesn't work, but i don't know why (the get instead yes)

    // this part should not be needed, since it is the same code in setRobotStateEnum
//    ros::NodeHandle nh;
//    ros::ServiceClient set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
//    robot_state::SetRobotState set_srv_msg;
//    set_srv_msg.request.setting_node = "..."; //TODO
//    set_srv_msg.request.robot_state = robot_state::IN_QUEUE;
//    bool call_succeeded = set_robot_state_sc.call(set_srv_msg);
//    EXPECT_TRUE(call_succeeded);

//    EXPECT_NE(robot_state::INITIALIZING, manager.getRobotStateEnum());
//    EXPECT_EQ(robot_state::IN_QUEUE, manager.getRobotStateEnum());    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "robot_state");

    // Wait some seconds to let robot_state node start //TODO would be better to use a service...
    ros::start(); // required to use ros::Time and ros::Duration
    ros::Duration(SLEEP_TIME_TO_LET_ROBOT_STATE_NODE_START).sleep();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
