#include <ros/ros.h>
#include <gtest/gtest.h>
#include "robot_state/robot_state_management.h"
#include "robot_state/SetRobotState.h"
#include "robot_state/GetRobotState.h"
#include "robot_state/TryToLockRobotState.h"
#include "robot_state/UnlockRobotState.h"

#define INVALID_STATE 1000000
#define SLEEP_TIME_TO_ALLOW_SERVICE_CREATION 1

//TODO dublicated code (for instance for declare the services)
TEST(RobotStateTest, testGetRobotStateService)
{
    ros::NodeHandle nh;
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    ros::Duration(SLEEP_TIME_TO_ALLOW_SERVICE_CREATION).sleep();

    robot_state::GetRobotState srv_msg;
    bool call_succeeded = get_robot_state_sc.call(srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::INITIALIZING, srv_msg.response.robot_state);
    EXPECT_NE(robot_state::CHARGING, srv_msg.response.robot_state);
}

TEST(RobotStateTest, testSetRobotStateService)
{
    bool call_succeeded;

    ros::NodeHandle nh;
    ros::ServiceClient set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    ros::Duration(SLEEP_TIME_TO_ALLOW_SERVICE_CREATION).sleep();

    robot_state::SetRobotState set_srv_msg;
    set_srv_msg.request.robot_state = robot_state::CHARGING;
    call_succeeded = set_robot_state_sc.call(set_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_TRUE(set_srv_msg.response.set_succeeded);

    robot_state::GetRobotState get_srv_msg;
    call_succeeded = get_robot_state_sc.call(get_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::CHARGING, get_srv_msg.response.robot_state);
}

TEST(RobotStateTest, testSetRobotStateServiceWithInvalidValue)
{
    bool call_succeeded;

    ros::NodeHandle nh;
    ros::ServiceClient set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    ros::Duration(SLEEP_TIME_TO_ALLOW_SERVICE_CREATION).sleep();

    robot_state::SetRobotState set_srv_msg;
    set_srv_msg.request.robot_state = INVALID_STATE;
    call_succeeded = set_robot_state_sc.call(set_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_FALSE(set_srv_msg.response.set_succeeded);
    
    robot_state::GetRobotState get_srv_msg;
    call_succeeded = get_robot_state_sc.call(get_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::CHARGING, get_srv_msg.response.robot_state); //NB: the test is the one set by the previous test case!!!
}

TEST(RobotStateTest, testLockAndUnlockServices)
{
    bool call_succeeded;

    ros::NodeHandle nh;
    ros::ServiceClient set_robot_state_sc = nh.serviceClient<robot_state::SetRobotState>("robot_state/set_robot_state");
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    ros::ServiceClient try_to_lock_robot_state_sc = nh.serviceClient<robot_state::TryToLockRobotState>("robot_state/try_to_lock_robot_state");
    ros::ServiceClient unlock_robot_state_sc = nh.serviceClient<robot_state::UnlockRobotState>("robot_state/unlock_robot_state");
    ros::Duration(SLEEP_TIME_TO_ALLOW_SERVICE_CREATION).sleep();

    robot_state::SetRobotState set_srv_msg;
    set_srv_msg.request.robot_state = robot_state::MOVING_TO_FRONTIER;
    call_succeeded = set_robot_state_sc.call(set_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_TRUE(set_srv_msg.response.set_succeeded);
    
    robot_state::GetRobotState get_srv_msg;
    call_succeeded = get_robot_state_sc.call(get_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::MOVING_TO_FRONTIER, get_srv_msg.response.robot_state);

    robot_state::TryToLockRobotState try_to_lock_srv_msg;
    try_to_lock_srv_msg.request.locking_node = "node_A";
    call_succeeded = try_to_lock_robot_state_sc.call(try_to_lock_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(true, try_to_lock_srv_msg.response.lock_acquired);

    try_to_lock_srv_msg.request.locking_node = "node_B";
    call_succeeded = try_to_lock_robot_state_sc.call(try_to_lock_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(false, try_to_lock_srv_msg.response.lock_acquired);

    set_srv_msg.request.setting_node = "node_B";
    set_srv_msg.request.robot_state = robot_state::CHOOSING_ACTION;
    call_succeeded = set_robot_state_sc.call(set_srv_msg);
    EXPECT_TRUE(call_succeeded);

    call_succeeded = get_robot_state_sc.call(get_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::MOVING_TO_FRONTIER, get_srv_msg.response.robot_state);
    EXPECT_NE(robot_state::CHOOSING_ACTION, get_srv_msg.response.robot_state);

    set_srv_msg.request.setting_node = "node_A";
    set_srv_msg.request.robot_state = robot_state::IN_QUEUE;
    call_succeeded = set_robot_state_sc.call(set_srv_msg);
    EXPECT_TRUE(call_succeeded);

    call_succeeded = get_robot_state_sc.call(get_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::IN_QUEUE, get_srv_msg.response.robot_state);
    EXPECT_NE(robot_state::MOVING_TO_FRONTIER, get_srv_msg.response.robot_state);

    robot_state::UnlockRobotState unlock_srv_msg;
    call_succeeded = unlock_robot_state_sc.call(unlock_srv_msg);
    EXPECT_TRUE(call_succeeded);

    set_srv_msg.request.setting_node = "node_B";
    set_srv_msg.request.robot_state = robot_state::CHOOSING_ACTION;
    call_succeeded = set_robot_state_sc.call(set_srv_msg);
    EXPECT_TRUE(call_succeeded);

    call_succeeded = get_robot_state_sc.call(get_srv_msg);
    EXPECT_TRUE(call_succeeded);
    EXPECT_EQ(robot_state::CHOOSING_ACTION, get_srv_msg.response.robot_state);
    EXPECT_NE(robot_state::IN_QUEUE, get_srv_msg.response.robot_state);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "robot_state");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
