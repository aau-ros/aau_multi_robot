#include <ros/ros.h>
#include <gtest/gtest.h>
#include <adhoc_communication/EmDockingStation.h>
#include <robot_state/robot_state_management.h>
#include "concrete_bid_computer.h"

#define ABSOLUTE_ERROR 0.001
#define SLEEP_TIME 0.1

double w1, w2, w3, w4;

TEST(TestConcreteBidComputer, testConstructorAndInitialization)
{
    ConcreteBidComputer cbc;
    cbc.updateLlh();
    EXPECT_NEAR(w3*1, cbc.getBid(), ABSOLUTE_ERROR);
}

TEST(TestConcreteBidComputer, testL1WithMoreVacantDssThanActiveRobots)
{
    ConcreteBidComputer cbc;

    ros::NodeHandle nh;
    ros::Publisher ds_pub = nh.advertise<adhoc_communication::EmDockingStation>("docking_stations", 1);
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("robots", 1);

    adhoc_communication::EmDockingStation ds_msg;
    ds_msg.vacant = true;
    ds_msg.id = 1;
    ds_pub.publish(ds_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.updateLlh();
    EXPECT_NEAR(w1*1 + w3*1, cbc.getBid(), ABSOLUTE_ERROR);

    adhoc_communication::EmRobot robot_msg;
    robot_msg.id = 11;
    robot_msg.state = robot_state::MOVING_TO_FRONTIER;
    robot_pub.publish(robot_msg);

    ds_msg.vacant = true;
    ds_msg.id = 2;
    ds_pub.publish(ds_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.updateLlh();
    EXPECT_NEAR(w1*1 + w3*1, cbc.getBid(), ABSOLUTE_ERROR);
}

TEST(TestConcreteBidComputer, testL1WithNoVacantDsAndNoActiveRobot)
{
    ConcreteBidComputer cbc;

    ros::NodeHandle nh;
    ros::Publisher ds_pub = nh.advertise<adhoc_communication::EmDockingStation>("docking_stations", 1);
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("robots", 1);

    adhoc_communication::EmDockingStation ds_msg;
    ds_msg.vacant = false;
    ds_msg.id = 1;
    ds_pub.publish(ds_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.updateLlh();
    EXPECT_NEAR(w3*1, cbc.getBid(), ABSOLUTE_ERROR);

    adhoc_communication::EmRobot robot_msg;
    robot_msg.id = 11;
    robot_msg.state = robot_state::IN_QUEUE;
    robot_pub.publish(robot_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.updateLlh();
    EXPECT_NEAR(w3*1, cbc.getBid(), ABSOLUTE_ERROR);
}

TEST(TestConcreteBidComputer, testL1ForCommonCase)
{
    ConcreteBidComputer cbc;

    ros::NodeHandle nh;
    ros::Publisher ds_pub = nh.advertise<adhoc_communication::EmDockingStation>("docking_stations", 1);
    ros::Publisher robot_pub = nh.advertise<adhoc_communication::EmRobot>("robots", 1);

    adhoc_communication::EmDockingStation ds_msg;
    ds_msg.vacant = true;
    ds_msg.id = 1;
    ds_pub.publish(ds_msg);
    ds_msg.vacant = false;
    ds_msg.id = 2;
    ds_pub.publish(ds_msg);
    ds_msg.vacant = true;
    ds_msg.id = 3;
    ds_pub.publish(ds_msg);

    adhoc_communication::EmRobot robot_msg;
    robot_msg.id = 11;
    robot_msg.state = robot_state::MOVING_TO_FRONTIER;
    robot_pub.publish(robot_msg);
    robot_msg.id = 22;
    robot_msg.state = robot_state::IN_QUEUE;
    robot_pub.publish(robot_msg);
    robot_msg.id = 33;
    robot_msg.state = robot_state::MOVING_TO_FRONTIER;
    robot_pub.publish(robot_msg);
    robot_msg.id = 44;
    robot_msg.state = robot_state::CHOOSING_ACTION;
    robot_pub.publish(robot_msg);
    robot_msg.id = 55;
    robot_msg.state = robot_state::COMPUTING_NEXT_GOAL;
    robot_pub.publish(robot_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.updateLlh();
    EXPECT_NEAR(w1*2.0/4.0 + w3*1, cbc.getBid(), ABSOLUTE_ERROR);

    ds_pub.publish(ds_msg);
    ds_msg.vacant = false;
    ds_msg.id = 3;
    ds_pub.publish(ds_msg);
    robot_msg.id = 55;
    robot_msg.state = robot_state::CHARGING;
    robot_pub.publish(robot_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.updateLlh();
    EXPECT_NEAR(w1*1.0/3.0 + w3*1, cbc.getBid(), ABSOLUTE_ERROR);
}

TEST(TestConcreteBidComputer, testL2WhenProcessMessagesIsNotCalled)
{
    ConcreteBidComputer cbc;

    ros::NodeHandle nh;
    ros::Publisher battery_pub = nh.advertise<explorer::battery_state>("battery_state", 1);

    explorer::battery_state msg;
    msg.remaining_time_run = 1.0;
    msg.remaining_time_charge = 2.0;
    battery_pub.publish(msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.updateLlh();
    EXPECT_NEAR(w3*1, cbc.getBid(), ABSOLUTE_ERROR);
}

TEST(TestConcreteBidComputer, testL2ForCommonCase)
{
    ConcreteBidComputer cbc;

    ros::NodeHandle nh;
    ros::Publisher battery_pub = nh.advertise<explorer::battery_state>("battery_state", 1);

    explorer::battery_state msg;
    msg.remaining_time_run = 1.0;
    msg.remaining_time_charge = 2.0;
    battery_pub.publish(msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.processMessages();
    cbc.updateLlh();
    EXPECT_NEAR(w2 * (2.0 / (1.0+2.0)) + w3*1, cbc.getBid(), ABSOLUTE_ERROR);
}

//TEST(TestConcreteBidComputer, testL3)
//{
//    ConcreteBidComputer cbc;

//    ros::NodeHandle nh;
//    ros::Publisher frontier_pub = nh.advertise<adhoc_communication::ExpFrontier>("frontiers", 1);
//    adhoc_communication::ExpFrontierElement elem;
//    adhoc_communication::ExpFrontier msg;
//    

//    ros::Duration(SLEEP_TIME).sleep();
//    ros::spinOnce();
//    cbc.processMessages();
//    cbc.updateLlh();
//    EXPECT_NEAR(w2 * (2.0 / (1.0+2.0)) + w3*1, cbc.getBid(), ABSOLUTE_ERROR);
//}

void ConcreteBidComputer::update_l3()
{
    message_mutex.lock();
    ROS_DEBUG("Update l3");
    
    unsigned int num_jobs, num_jobs_close;
    countJobsAndCloseJobs(num_jobs, num_jobs_close);

    if (num_jobs_close > num_jobs)
    {
        ROS_ERROR("Number of jobs close by greater than total number of jobs: %d > %d", num_jobs_close, num_jobs);
        l3 = 0;
    }
    else {
        if (num_jobs == 0)
            l3 = 1;
        else
            l3 = (num_jobs - num_jobs_close) / num_jobs;
        ROS_DEBUG("l3: %.1f", l3);
    }

    message_mutex.unlock();
}


int main(int argc, char **argv){
    ros::init(argc, argv, "concrete_bid_computer_utest");

    ros::NodeHandle nh_tilde("~");
    nh_tilde.getParam("w1", w1);
    nh_tilde.getParam("w2", w2);
    nh_tilde.getParam("w3", w3);
    nh_tilde.getParam("w4", w4);

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
