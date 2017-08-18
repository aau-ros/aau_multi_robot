#include <ros/ros.h>
#include <gtest/gtest.h>
#include <adhoc_communication/EmDockingStation.h>
#include <adhoc_communication/ExpFrontierElement.h>
#include <adhoc_communication/ExpFrontier.h>
#include <robot_state/robot_state_management.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

TEST(TestConcreteBidComputer, testL3ForCommonCase)
{
    ConcreteBidComputer cbc;

    ros::NodeHandle nh;
    ros::Publisher frontier_pub = nh.advertise<adhoc_communication::ExpFrontier>("frontiers", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);
    ros::Publisher battery_pub = nh.advertise<explorer::battery_state>("battery_state", 1);
    ros::Publisher optimal_ds_pub = nh.advertise<adhoc_communication::EmDockingStation>("explorer/new_optimal_ds", 1);

    adhoc_communication::EmDockingStation optimal_ds_msg;
    optimal_ds_msg.x = -10;
    optimal_ds_msg.y = -10;
    optimal_ds_pub.publish(optimal_ds_msg);

    explorer::battery_state msg;
    msg.remaining_time_run = 1.0;
    msg.remaining_time_charge = 0.0;
    msg.maximum_traveling_distance = 50;
    battery_pub.publish(msg);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.pose.pose.position.x = 10;
    pose_msg.pose.pose.position.y = -10;
    pose_pub.publish(pose_msg);

    adhoc_communication::ExpFrontierElement frontier;
    adhoc_communication::ExpFrontier frontier_list_msg;
    frontier.x_coordinate = 0;
    frontier.y_coordinate = -20;
    frontier_list_msg.frontier_element.push_back(frontier);
    frontier_pub.publish(frontier_list_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.processMessages();
    cbc.updateLlh();
    EXPECT_NEAR(0, cbc.getBid(), ABSOLUTE_ERROR);

    frontier.x_coordinate = 100;
    frontier.y_coordinate = -100;
    frontier_list_msg.frontier_element.push_back(frontier);
    frontier.x_coordinate = 30;
    frontier.y_coordinate = -10;
    frontier_list_msg.frontier_element.push_back(frontier);
    frontier_pub.publish(frontier_list_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.processMessages();
    cbc.updateLlh();
    EXPECT_NEAR(w3*(2.0/3.0), cbc.getBid(), ABSOLUTE_ERROR);

    optimal_ds_msg.x = -1000;
    optimal_ds_msg.y = -1000;
    optimal_ds_pub.publish(optimal_ds_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.processMessages();
    cbc.updateLlh();
    EXPECT_NEAR(w3*1, cbc.getBid(), ABSOLUTE_ERROR);
}

TEST(TestConcreteBidComputer, testL4ForCommonCase)
{
    ConcreteBidComputer cbc;

    ros::NodeHandle nh;
    ros::Publisher frontier_pub = nh.advertise<adhoc_communication::ExpFrontier>("frontiers", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1);
    ros::Publisher battery_pub = nh.advertise<explorer::battery_state>("battery_state", 1);
    ros::Publisher optimal_ds_pub = nh.advertise<adhoc_communication::EmDockingStation>("explorer/new_optimal_ds", 1);
    ros::Publisher ds_pub = nh.advertise<adhoc_communication::EmDockingStation>("docking_stations", 1);

    adhoc_communication::EmDockingStation ds_msg;
    ds_msg.x = -10;
    ds_msg.y = -10;
    optimal_ds_pub.publish(ds_msg);
    ds_pub.publish(ds_msg);

    explorer::battery_state msg;
    msg.remaining_time_run = 1.0;
    msg.remaining_time_charge = 0.0;
    msg.maximum_traveling_distance = 50;
    battery_pub.publish(msg);

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.pose.pose.position.x = 10;
    pose_msg.pose.pose.position.y = -10;
    pose_pub.publish(pose_msg);

    adhoc_communication::ExpFrontierElement frontier;
    adhoc_communication::ExpFrontier frontier_list_msg;
    frontier.x_coordinate = 1000;
    frontier.y_coordinate = 1000;
    frontier_list_msg.frontier_element.push_back(frontier);
    frontier.x_coordinate = 110;
    frontier.y_coordinate = -10;
    frontier_list_msg.frontier_element.push_back(frontier);
    frontier_pub.publish(frontier_list_msg);

    ros::Duration(SLEEP_TIME).sleep();
    ros::spinOnce();
    cbc.processMessages();
    cbc.updateLlh();
    EXPECT_NEAR(w3*1 + w4*100.0/(100.0+20.0), cbc.getBid(), ABSOLUTE_ERROR);
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
