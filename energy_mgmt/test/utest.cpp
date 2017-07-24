#include "docking.h"
#include "battery_simulate.h"
#include "mock_time_manager.h"
#include <ros/ros.h>
#include <gtest/gtest.h>

#define SENDING_SLEEP_TIME 1
#define FLOAT_ABSOLUTE_ERROR 0.001

//TODO load from launch file 
const double power_charging =             300;
const double power_moving_fixed_cost =     10;
const double power_per_speed =              8;
const double power_microcontroller =       40;
const double power_sonar =                  5;
const double power_laser =                 15;
const double power_basic_computations =    20;
const double power_advanced_computations = 30;
const double maximum_traveling_distance = 100;

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

enum state_t
    {
        exploring,  // the robot is computing which is the next frontier to be
                    // explored

        going_charging,  // the robot has the right to occupy a DS to recharge

        charging,  // the robot is charging at a DS

        finished,  // the robot has finished the exploration

        fully_charged,  // the robot has recently finished a charging process; notice
                        // that the robot is in this state even if it is not really
                        // fully charged (since just after a couple of seconds after
                        // the end of the recharging process the robot has already
                        // lost some battery energy, since it consumes power even
                        // when it stays still

        stuck,

        in_queue,  // the robot is in a queue, waiting for a DS to be vacant

        auctioning,  // auctioning: the robot has started an auction; notice that if
                     // the robot is aprticipating to an auction that it was not
                     // started by it, its state is not equal to auctioning!!!
                     
        auctioning_2,

        going_in_queue,  // the robot is moving near a DS to later put itself in
                         // in_queue state

        going_checking_vacancy,  // the robot is moving near a DS to check if it
                                 // vacant, so that it can occupy it and start
                                 // recharging

        checking_vacancy,  // the robot is currently checking if the DS is vacant,
                           // i.e., it is waiting information from the other robots
                           // about the state of the DS

        moving_to_frontier_before_going_charging,  // TODO hmm...

        moving_to_frontier,  // the robot has selected the next frontier to be
                             // reached, and it is moving toward it
        leaving_ds,          // the robot was recharging, but another robot stopped
        dead,
        moving_away_from_ds,
        auctioning_3
    };

TEST(TestSuite, testCase2)
{
    MockTimeManager mtm;
    battery_simulate bat;
    bat.setTimeManager(&mtm);
    EXPECT_EQ(bat.getMaximumTravelingDistance(), maximum_traveling_distance); //TODO in many places expected and actual values are inverted in EXPECT_EQ
}

// check current time
TEST(TestSuite, testCase3)
{
    MockTimeManager mtm;
    double time = 30;
    mtm.addTime(time);
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.set_last_time();
    EXPECT_EQ(bat.last_time_secs(), time);
}

TEST(TestSuite, testCase4)
{  
    battery_simulate bat;
    bat.initializing = false;
    double consumed_energy = bat.getConsumedEnergyA() + bat.getConsumedEnergyB();
    EXPECT_EQ(consumed_energy, 0);
}

TEST(TestSuite, testCase5)
{
    MockTimeManager mtm;
    double time1 = 30, time2 = 70, time3 = 120;
    mtm.addTime(time1);
    mtm.addTime(time2);
    mtm.addTime(time3);
    
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.initializeSimulationTime();
    
    EXPECT_EQ(time1, bat.last_time_secs());
    bat.compute();
    EXPECT_EQ(time3, bat.last_time_secs());
    EXPECT_EQ(40, bat.getElapsedTime());
}

TEST(TestSuite, testCase6)
{
    MockTimeManager mtm;
    mtm.addTime(0);
    mtm.addTime(30);
    double test_time = 70;
    mtm.addTime(test_time);
    mtm.addTime(100);
    
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.initializeSimulationTime();
    bat.compute();

    EXPECT_EQ(test_time, bat.last_time_secs());
}

TEST(TestSuite, testCase7)
{
    MockTimeManager mtm;
    double time1 = 30, time2 = 70, time3 = 120, time4 = 200, time5 = 250;
    mtm.addTime(time1);
    mtm.addTime(time2);
    mtm.addTime(time3);
    mtm.addTime(time4);
    mtm.addTime(time5);
    
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.initializeSimulationTime();
    
    ros::NodeHandle nh;
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 0;
    cmd_vel_pub.publish(cmd_vel_msg);
    
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("explorer/robot", 10, true);
    adhoc_communication::EmRobot robot_state_msg;
    robot_state_msg.state = exploring;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    bat.spinOnce();
    
    double consumed_energy, expected_consumed_energy;
    bat.compute();
    consumed_energy = bat.getConsumedEnergyB();
    expected_consumed_energy = (power_sonar + power_laser + power_microcontroller + power_basic_computations + power_advanced_computations) * (time2 - time1);
    EXPECT_EQ(time3, bat.last_time_secs());
    EXPECT_EQ(time2 - time1, bat.getElapsedTime());
    EXPECT_EQ(consumed_energy, expected_consumed_energy);
    EXPECT_EQ(bat.getConsumedEnergyA(), 0);
    
    bat.compute();
    EXPECT_EQ(time5, bat.last_time_secs());
    EXPECT_EQ(time4 - time3, bat.getElapsedTime());
    expected_consumed_energy += (power_sonar + power_laser + power_microcontroller + power_basic_computations + power_advanced_computations) * (time4 - time3);
    consumed_energy = bat.getConsumedEnergyA() + bat.getConsumedEnergyB();
    EXPECT_EQ(consumed_energy, expected_consumed_energy);
    EXPECT_EQ(bat.getConsumedEnergyA(), 0);
}

TEST(TestSuite, testCase8)
{
    MockTimeManager mtm;
    double time1 = 30, time2 = 70, time3 = 120, time4 = 200, time5 = 250;
    mtm.addTime(time1);
    mtm.addTime(time2);
    mtm.addTime(time3);
    mtm.addTime(time4);
    mtm.addTime(time5);
    
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.initializeSimulationTime();
    
    ros::NodeHandle nh;
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    geometry_msgs::Twist cmd_vel_msg;
    double speed = 0;
    cmd_vel_msg.linear.x = speed;
    cmd_vel_pub.publish(cmd_vel_msg);
    
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("explorer/robot", 10, true);
    adhoc_communication::EmRobot robot_state_msg;
    robot_state_msg.state = moving_to_frontier;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep(); // necessary, or the messages are not received in time by the battery manager
    bat.spinOnce();
    
    double consumed_energy, expected_consumed_energy;
    bat.compute();
    consumed_energy = bat.getConsumedEnergyB();
    expected_consumed_energy = (power_sonar + power_laser + power_microcontroller + power_basic_computations + power_advanced_computations) * (time2 - time1);
    EXPECT_EQ(time3, bat.last_time_secs());
    EXPECT_EQ(time2 - time1, bat.getElapsedTime());
    EXPECT_EQ(consumed_energy, expected_consumed_energy);
    EXPECT_EQ(bat.getConsumedEnergyA(), 0);
    
    speed = 10;
    cmd_vel_msg.linear.x = speed;
    cmd_vel_pub.publish(cmd_vel_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    bat.spinOnce();
    
    bat.compute();
    EXPECT_EQ(time5, bat.last_time_secs());
    EXPECT_EQ(time4 - time3, bat.getElapsedTime());
    expected_consumed_energy += (power_per_speed * speed + power_moving_fixed_cost + power_sonar + power_laser + power_microcontroller + power_basic_computations + power_advanced_computations) * (time4 - time3);
    consumed_energy = bat.getConsumedEnergyA() + bat.getConsumedEnergyB();
    EXPECT_EQ(consumed_energy, expected_consumed_energy);
}

// exploring + charging
TEST(TestSuite, testCase9)
{
    MockTimeManager mtm;
    double time1 = 0, time2 = 500, time3 = 800, time4 = 1000, time5 = 1250;
    mtm.addTime(time1);
    mtm.addTime(time2);
    mtm.addTime(time3);
    mtm.addTime(time4);
    mtm.addTime(time5);
    
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.initializeSimulationTime();
    
    ros::NodeHandle nh;
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    geometry_msgs::Twist cmd_vel_msg;
    double speed = 15;
    cmd_vel_msg.linear.x = speed;
    cmd_vel_pub.publish(cmd_vel_msg);
    
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("explorer/robot", 10, true);
    adhoc_communication::EmRobot robot_state_msg;
    robot_state_msg.state = exploring;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep(); // necessary, or the messages are not received in time by the battery manager
    bat.spinOnce();
    
    double consumed_energy_A, consumed_energy_B, expected_consumed_energy_A, expected_consumed_energy_B;
    bat.compute();
    consumed_energy_A = bat.getConsumedEnergyA();
    consumed_energy_B = bat.getConsumedEnergyB();
    expected_consumed_energy_A = (power_moving_fixed_cost + power_per_speed * speed) * (time2 - time1);
    expected_consumed_energy_B = (power_sonar + power_laser + power_microcontroller + power_basic_computations + power_advanced_computations) * (time2 - time1);
    EXPECT_EQ(time3, bat.last_time_secs());
    EXPECT_EQ(time2 - time1, bat.getElapsedTime());
    EXPECT_EQ(consumed_energy_A, expected_consumed_energy_A);
    EXPECT_EQ(consumed_energy_B, expected_consumed_energy_B);
    
    robot_state_msg.state = charging;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    bat.spinOnce();
    
    bat.compute();
    EXPECT_EQ(time5, bat.last_time_secs());
    EXPECT_EQ(time4 - time3, bat.getElapsedTime());
    EXPECT_GT(consumed_energy_A, consumed_energy_B);
    
    expected_consumed_energy_A -= consumed_energy_A / (consumed_energy_A + consumed_energy_B) * power_charging * (time4 - time3);
    expected_consumed_energy_B -= consumed_energy_B / (consumed_energy_A + consumed_energy_B) * power_charging * (time4 - time3);
    expected_consumed_energy_B += (power_microcontroller + power_basic_computations) * (time4 - time3);
    consumed_energy_A = bat.getConsumedEnergyA();
    consumed_energy_B = bat.getConsumedEnergyB();
    EXPECT_NEAR(consumed_energy_A, expected_consumed_energy_A, FLOAT_ABSOLUTE_ERROR);
    EXPECT_NEAR(consumed_energy_B, expected_consumed_energy_B, FLOAT_ABSOLUTE_ERROR);
}

TEST(TestSuite, testCase10)
{
    MockTimeManager mtm;
    double time1 = 0, time2 = 1000, time3 = 1250;
    mtm.addTime(time1);
    mtm.addTime(time2);
    mtm.addTime(time3);
    
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.initializeSimulationTime();
    
    ros::NodeHandle nh;
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = 10;
    cmd_vel_pub.publish(cmd_vel_msg);
    
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("explorer/robot", 10, true);
    adhoc_communication::EmRobot robot_state_msg;
    
    robot_state_msg.state = exploring;
    robot_pub.publish(robot_state_msg);
    
    robot_state_msg.state = in_queue;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep(); // necessary, or the messages are not received in time by the battery manager
    bat.spinOnce();
    
    double consumed_energy, expected_consumed_energy;
    bat.compute();
    consumed_energy = bat.getConsumedEnergyA() + bat.getConsumedEnergyB();
    expected_consumed_energy = (power_microcontroller + power_basic_computations) * (time2 - time1);
    EXPECT_EQ(time3, bat.last_time_secs());
    EXPECT_EQ(time2 - time1, bat.getElapsedTime());
    EXPECT_EQ(consumed_energy, expected_consumed_energy);
}

TEST(TestSuite, testCase11)
{
    MockTimeManager mtm;
    double time1 = 0, time2 = 500, time3 = 1000, time4 = 2000, time5 = 5000;
    mtm.addTime(time1);
    mtm.addTime(time2);
    mtm.addTime(time3);
    mtm.addTime(time4);
    mtm.addTime(time5);
    
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.initializeSimulationTime();
    
    ros::NodeHandle nh;
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    geometry_msgs::Twist cmd_vel_msg;
    double speed = 15;
    cmd_vel_msg.linear.x = speed;
    cmd_vel_pub.publish(cmd_vel_msg);
    
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("explorer/robot", 10, true);
    adhoc_communication::EmRobot robot_state_msg;
    robot_state_msg.state = exploring;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep(); // necessary, or the messages are not received in time by the battery manager
    bat.spinOnce();
    
    double consumed_energy_A, consumed_energy_B, expected_consumed_energy_A, expected_consumed_energy_B;
    bat.compute();
    consumed_energy_A = bat.getConsumedEnergyA();
    consumed_energy_B = bat.getConsumedEnergyB();
    expected_consumed_energy_A = (power_moving_fixed_cost + power_per_speed * speed) * (time2 - time1);
    expected_consumed_energy_B = (power_sonar + power_laser + power_microcontroller + power_basic_computations + power_advanced_computations) * (time2 - time1);
    EXPECT_EQ(time3, bat.last_time_secs());
    EXPECT_EQ(time2 - time1, bat.getElapsedTime());
    EXPECT_EQ(expected_consumed_energy_A, consumed_energy_A);
    EXPECT_EQ(expected_consumed_energy_B, consumed_energy_B);
    
    robot_state_msg.state = charging;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    bat.spinOnce();
    
    bat.compute();
    
    EXPECT_EQ(time5, bat.last_time_secs());
    EXPECT_EQ(time4 - time3, bat.getElapsedTime());
    expected_consumed_energy_A = 0;
    expected_consumed_energy_B = 0;
    consumed_energy_A = bat.getConsumedEnergyA();
    consumed_energy_B = bat.getConsumedEnergyB();
    
    EXPECT_NEAR(expected_consumed_energy_A, consumed_energy_A, FLOAT_ABSOLUTE_ERROR);
    EXPECT_NEAR(expected_consumed_energy_B, consumed_energy_B, FLOAT_ABSOLUTE_ERROR);
     
}

TEST(TestSuite, testCase12)
{
    MockTimeManager mtm;
    double time1 = 0, time2 = 500, time3 = 800, time4 = 1000, time5 = 1250;
    mtm.addTime(time1);
    mtm.addTime(time2);
    mtm.addTime(time3);
    mtm.addTime(time4);
    mtm.addTime(time5);
    
    battery_simulate bat;
    bat.initializing = false;
    bat.setTimeManager(&mtm);
    bat.initializeSimulationTime();
    
    ros::NodeHandle nh;
    
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
    geometry_msgs::Twist cmd_vel_msg;
    double speed = 0.001;
    cmd_vel_msg.linear.x = speed;
    cmd_vel_pub.publish(cmd_vel_msg);
    
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("explorer/robot", 10, true);
    adhoc_communication::EmRobot robot_state_msg;
    robot_state_msg.state = exploring;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep(); // necessary, or the messages are not received in time by the battery manager
    bat.spinOnce();
    
    double consumed_energy_A, consumed_energy_A2, consumed_energy_B, expected_consumed_energy_A, expected_consumed_energy_B;
    bat.compute();
    consumed_energy_A = bat.getConsumedEnergyA();
    consumed_energy_B = bat.getConsumedEnergyB();
    expected_consumed_energy_A = (power_moving_fixed_cost + power_per_speed * speed) * (time2 - time1);
    expected_consumed_energy_B = (power_sonar + power_laser + power_microcontroller + power_basic_computations + power_advanced_computations) * (time2 - time1);
    EXPECT_EQ(time3, bat.last_time_secs());
    EXPECT_EQ(time2 - time1, bat.getElapsedTime());
    EXPECT_EQ(consumed_energy_A, expected_consumed_energy_A);
    EXPECT_EQ(consumed_energy_B, expected_consumed_energy_B);
    
    robot_state_msg.state = charging;
    robot_pub.publish(robot_state_msg);
    
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    bat.spinOnce();
    
    bat.compute();
    EXPECT_EQ(time5, bat.last_time_secs());
    EXPECT_EQ(time4 - time3, bat.getElapsedTime());
    expected_consumed_energy_A -= consumed_energy_A / (consumed_energy_A + consumed_energy_B) * power_charging * (time4 - time3);
    expected_consumed_energy_B -= consumed_energy_B / (consumed_energy_A + consumed_energy_B) * power_charging * (time4 - time3);
    expected_consumed_energy_B += (power_microcontroller + power_basic_computations) * (time4 - time3);
    consumed_energy_A2 = bat.getConsumedEnergyA();
    consumed_energy_B = bat.getConsumedEnergyB();

    
    EXPECT_NEAR(consumed_energy_A2, expected_consumed_energy_A, FLOAT_ABSOLUTE_ERROR);
    EXPECT_NEAR(consumed_energy_B, expected_consumed_energy_B, FLOAT_ABSOLUTE_ERROR);
    
    double excected_remaining_distance = (consumed_energy_A - consumed_energy_A2) / consumed_energy_A * maximum_traveling_distance;
    EXPECT_NEAR(bat.getRemainingDistance(), excected_remaining_distance, FLOAT_ABSOLUTE_ERROR);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "energy_mgmt");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
