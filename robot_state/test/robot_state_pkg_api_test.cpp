#include <ros/ros.h>
#include <gtest/gtest.h>
#include "robot_state/robot_state_management.h"
#include <robot_state/GetRobotState.h>

#define INVALID_STATE 1000000
#define SLEEP_TIME_TO_ALLOW_SERVICE_CREATION 1

//TODO dublicated code (for instance for declare the services)
TEST(RobotStatePkgApiTest, testGetRobotState)
{
//    RobotStateApi manager;
//    ros::Duration(10).sleep();
//    EXPECT_TRUE(manager.getRobotState2());


    robot_state::GetRobotState get_srv_msg;
    ros::NodeHandle nh;
    ros::ServiceClient get_robot_state_sc = nh.serviceClient<robot_state::GetRobotState>("robot_state/get_robot_state");
    EXPECT_TRUE(get_robot_state_sc.call(get_srv_msg));


//    bool succees = (InitializingState * s = dynamic_cast<InitializingState*>(manager.getRobotState()));
//    EXPECT_TRUE(succees);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "robot_state");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
