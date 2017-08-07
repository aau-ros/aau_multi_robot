#include <ros/ros.h>
#include <gtest/gtest.h>
#include "robot_state/robot_state.h"
#include "robot_state/SetRobotState.h"
#include "robot_state/GetRobotState.h"

#define INVALID_STATE 1000000

TEST(RobotStateTest, testGetRobotStateService)
{
    ros::NodeHandle nh;
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    robot_state::GetRobotState srv_msg;
    bool call_succeeded = get_robot_state_sc.call(srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::INITIALIZING, srv_msg.response.robot_state);
    EXPECT_NE(robot_state::COMPUTING, srv_msg.response.robot_state);
}

TEST(RobotStateTest, testSetRobotStateService)
{
    bool call_succeeded;

    ros::NodeHandle nh;
    ros::ServiceClient set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");

    robot_state::SetRobotState set_srv_msg;
    set_srv_msg.request.robot_state = robot_state::COMPUTING;
    call_succeeded = set_robot_state_sc.call(set_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_TRUE(set_srv_msg.response.set_succeeded);

    robot_state::GetRobotState get_srv_msg;
    call_succeeded = get_robot_state_sc.call(get_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::COMPUTING, get_srv_msg.response.robot_state);
}

TEST(RobotStateTest, testSetRobotStateServiceWithInvalidValue)
{
    bool call_succeeded;

    ros::NodeHandle nh;
    ros::ServiceClient set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");

    robot_state::SetRobotState set_srv_msg;
    set_srv_msg.request.robot_state = INVALID_STATE;
    call_succeeded = set_robot_state_sc.call(set_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_FALSE(set_srv_msg.response.set_succeeded);
    
    robot_state::GetRobotState get_srv_msg;
    call_succeeded = get_robot_state_sc.call(get_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::COMPUTING, get_srv_msg.response.robot_state); //NB: the test is the one set by the previous test case!!!
}

int main(int argc, char **argv){
  ros::init(argc, argv, "robot_state");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
