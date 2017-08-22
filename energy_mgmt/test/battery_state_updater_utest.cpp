#include <ros/ros.h>
#include <gtest/gtest.h>
#include <explorer/battery_state.h>
#include <utilities/mock_time_manager.h>
#include "battery_state_updater.h"
#include "robot_state_manager2.h"

#define SENDING_SLEEP_TIME 0.1
#define FLOAT_ABSOLUTE_ERROR 0.001

double power_charging;
double power_moving_fixed_cost;
double power_per_speed;
double power_microcontroller;
double power_sonar;
double power_laser;
double power_basic_computations;
double power_advanced_computations;
double maximum_traveling_distance;
double max_speed_linear;
double speed_avg_init;
std::string robot_prefix;
std::string log_path;

namespace testing
{
 namespace internal
 {
  enum GTestColor {
      COLOR_DEFAULT,
      COLOR_RED,
      COLOR_GREEN,
      COLOR_YELLOW
  };

  extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
 }
}
#define PRINTF(...)  do { testing::internal::ColoredPrintf(testing::internal::COLOR_GREEN, "[          ] "); testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); } while(0)

// C++ stream interface
class TestCout : public std::stringstream
{
public:
    ~TestCout()
    {
        PRINTF("%s",str().c_str());
    }
};

#define TEST_COUT  TestCout()

TEST(TestSuite, testUpdateInInitializingState)
{
    MockTimeManager mtm;
    explorer::battery_state bt;
    BatteryStateUpdater bsu(&bt);
    RobotStateManager2 rsm;
    bsu.setTimeManager(&mtm);
    bsu.setRobotStateManager(&rsm);

    rsm.setRobotState(robot_state::INITIALIZING);
    mtm.addTime(100);
    bsu.updateBatteryState();
    EXPECT_EQ(100 * (power_microcontroller + power_basic_computations), bt.consumed_energy_B);
}

TEST(TestSuite, testUpdateInMovingToFrontierState)
{
    MockTimeManager mtm;
    explorer::battery_state bt;

    BatteryStateUpdater bsu(&bt);
    RobotStateManager2 rsm;
    bsu.setTimeManager(&mtm);
    bsu.setRobotStateManager(&rsm);

    rsm.setRobotState(robot_state::MOVING_TO_FRONTIER);
    mtm.addTime(150);
    bt.consumed_energy_B = 30;
    bt.consumed_energy_A = 50;

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Publisher amcl_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);

    geometry_msgs::Twist msg;
    msg.linear.x = 11;
    cmd_vel_pub.publish(msg);

    ros::Duration(SENDING_SLEEP_TIME).sleep();
    ros::spinOnce();
    bsu.updateBatteryState();
    double expected_consumed_energy_A = 50 + 150 * (power_per_speed * 11 + power_moving_fixed_cost);
    double expected_consumed_energy_B = 30 + 150 * (power_microcontroller + power_basic_computations + power_sonar + power_laser);
    EXPECT_EQ(expected_consumed_energy_A, bt.consumed_energy_A);
    EXPECT_EQ(expected_consumed_energy_B, bt.consumed_energy_B);
    EXPECT_EQ(maximum_traveling_distance, bt.remaining_distance);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.pose.pose.position.x = 10;
    pose_msg.pose.pose.position.y = 10;
    amcl_pose_pub.publish(pose_msg);
    pose_msg.pose.pose.position.x = 10;
    pose_msg.pose.pose.position.y = 15;
    amcl_pose_pub.publish(pose_msg);
    pose_msg.pose.pose.position.x = 15;
    pose_msg.pose.pose.position.y = 15;
    amcl_pose_pub.publish(pose_msg);

    mtm.addTime(150+10);    
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    ros::spinOnce();
    bsu.updateBatteryState();

    expected_consumed_energy_A = expected_consumed_energy_A + 10 * (power_per_speed * 11 + power_moving_fixed_cost);
    expected_consumed_energy_B = expected_consumed_energy_B + 10 * (power_microcontroller + power_basic_computations + power_sonar + power_laser);
    EXPECT_EQ(expected_consumed_energy_A, bt.consumed_energy_A);
    EXPECT_EQ(expected_consumed_energy_B, bt.consumed_energy_B);
    double remaining_distance = maximum_traveling_distance - (10*sqrt(2) + 5 + 5);
    EXPECT_EQ(expected_consumed_energy_A, bt.consumed_energy_A);
    EXPECT_EQ(expected_consumed_energy_B, bt.consumed_energy_B);
    EXPECT_NEAR(remaining_distance, bt.remaining_distance, FLOAT_ABSOLUTE_ERROR);

    mtm.addTime(150+10+30);
    msg.linear.x = 5.5;
    cmd_vel_pub.publish(msg);
    pose_msg.pose.pose.position.x = 22.5;
    pose_msg.pose.pose.position.y = 15;
    amcl_pose_pub.publish(pose_msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    ros::spinOnce();
    bsu.updateBatteryState();
    remaining_distance = remaining_distance - 7.5;
    expected_consumed_energy_A = expected_consumed_energy_A + 30 * (power_per_speed * 5.5 + power_moving_fixed_cost);
    expected_consumed_energy_B = expected_consumed_energy_B + 30 * (power_microcontroller + power_basic_computations + power_sonar + power_laser);
    EXPECT_NEAR(expected_consumed_energy_A, bt.consumed_energy_A, FLOAT_ABSOLUTE_ERROR);
    EXPECT_EQ(expected_consumed_energy_B, bt.consumed_energy_B);
    EXPECT_NEAR(remaining_distance, bt.remaining_distance, FLOAT_ABSOLUTE_ERROR);
}

TEST(TestSuite, testUpdateInComputingNextGoalState)
{
    MockTimeManager mtm;
    explorer::battery_state bt;
    BatteryStateUpdater bsu(&bt);
    RobotStateManager2 rsm;
    bsu.setTimeManager(&mtm);
    bsu.setRobotStateManager(&rsm);

    rsm.setRobotState(robot_state::COMPUTING_NEXT_GOAL);
    mtm.addTime(100);
    bsu.updateBatteryState();
    EXPECT_EQ(100 * (power_microcontroller + power_basic_computations + power_advanced_computations), bt.consumed_energy_B);
}

TEST(TestSuite, testUpdateInChargingState)
{
    MockTimeManager mtm;
    explorer::battery_state bt;
    BatteryStateUpdater bsu(&bt);
    RobotStateManager2 rsm;
    bsu.setTimeManager(&mtm);
    bsu.setRobotStateManager(&rsm);

    bt.consumed_energy_A = 2000;
    bt.remaining_distance = 5;

    rsm.setRobotState(robot_state::INITIALIZING);
    mtm.addTime(0);
    bsu.updateBatteryState();

    bt.consumed_energy_B = 10000;

    rsm.setRobotState(robot_state::CHARGING);
    mtm.addTime(10);

    double consumed_energy_B_after_update = 10000.0 + 10.0 * (power_microcontroller + power_basic_computations);
    double ratio_A = 2000.0 / ( consumed_energy_B_after_update + 2000.0);
    double ratio_B = consumed_energy_B_after_update / ( consumed_energy_B_after_update + 2000.0);
    double consumed_energy_A_after_charge = 2000 - 10 * power_charging * ratio_A;
    double consumed_energy_B_after_charge = consumed_energy_B_after_update - 10 * power_charging * ratio_B;

    mtm.addTime(12.5);

    bsu.updateBatteryState();
    EXPECT_NEAR(consumed_energy_A_after_charge, bt.consumed_energy_A, FLOAT_ABSOLUTE_ERROR);
    EXPECT_NEAR(consumed_energy_B_after_charge, bt.consumed_energy_B, FLOAT_ABSOLUTE_ERROR);
    EXPECT_NEAR((2000 - consumed_energy_A_after_charge) / 2000 * maximum_traveling_distance, bt.remaining_distance, FLOAT_ABSOLUTE_ERROR);

    double consumed_energy_B_after_update_2 = consumed_energy_B_after_charge + 2.5 * (power_microcontroller + power_basic_computations);
    double ratio_A_2 = consumed_energy_A_after_charge / ( consumed_energy_B_after_update_2 + consumed_energy_A_after_charge);
    double ratio_B_2 = consumed_energy_B_after_update_2 / ( consumed_energy_B_after_update_2 + consumed_energy_A_after_charge);
    double consumed_energy_A_after_charge_2 = consumed_energy_A_after_charge - 2.5 * power_charging * ratio_A_2;
    double consumed_energy_B_after_charge_2 = consumed_energy_B_after_update_2 - 2.5 * power_charging * ratio_B_2;

    bsu.updateBatteryState();
    EXPECT_NEAR(consumed_energy_A_after_charge_2, bt.consumed_energy_A, FLOAT_ABSOLUTE_ERROR);
    EXPECT_NEAR(consumed_energy_B_after_charge_2, bt.consumed_energy_B, FLOAT_ABSOLUTE_ERROR);
    EXPECT_NEAR((2000 - consumed_energy_A_after_charge_2) / 2000 * maximum_traveling_distance, bt.remaining_distance, FLOAT_ABSOLUTE_ERROR);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "battery_state_updater_utest");

    ros::NodeHandle nh_tilde("~");
    if(!nh_tilde.getParam("speed_avg_init", speed_avg_init)) //TODO use config file instead of yaml file for the parameters
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("power_charging", power_charging))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("power_per_speed", power_per_speed))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("power_moving_fixed_cost", power_moving_fixed_cost))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("power_sonar", power_sonar))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("power_laser", power_laser))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("power_microcontroller", power_microcontroller))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("power_basic_computations", power_basic_computations))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("power_advanced_computations", power_advanced_computations))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("max_linear_speed", max_speed_linear))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("maximum_traveling_distance", maximum_traveling_distance))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("log_path", log_path))
        ROS_FATAL("INVALID PARAM");
    if(!nh_tilde.getParam("robot_prefix", robot_prefix))
        ROS_FATAL("INVALID PARAM");

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
