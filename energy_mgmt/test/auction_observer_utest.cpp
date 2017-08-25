#include <ros/ros.h>
#include <gtest/gtest.h>
#include <adhoc_communication/EmDockingStation.h>
#include <utilities/mock_time_manager.h>
#include "auction_observer.h"
#include "robot_state_manager2.h"
#include "mock_auction_manager.h"

double reauctioning_timeout;

TEST(TestAuctionObserver, testUnableToCompleteAuciton)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    MockTimeManager mtm;
    mtm.addTime(200);
    mtm.addTime(200);
    mtm.addTime(200);
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);
    mam.current_auction_test.starting_time = 10;
    mam.current_auction_test.ending_time = -1;
    ao.sanityChecks();
    EXPECT_TRUE(ao.error_1);
}

TEST(TestAuctionObserver, testAbleToCompleteAuciton)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    MockTimeManager mtm;
    mtm.addTime(15);
    mtm.addTime(15);
    mtm.addTime(15);
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);
    mam.current_auction_test.starting_time = 10;
    mam.current_auction_test.ending_time = 20;
    ao.sanityChecks();
    EXPECT_FALSE(ao.error_1);
}

TEST(TestAuctionObserver, testUnableToStartAuction)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    MockTimeManager mtm;
    mtm.addTime(1000);
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);
    mam.current_auction_test.starting_time = -1;
    ao.sanityChecks();
    EXPECT_TRUE(ao.error_2);
}

TEST(TestAuctionObserver, testStuckInQueue)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    mam.participating_test = false;
    MockTimeManager mtm;
    rsm.setRobotState(robot_state::IN_QUEUE);
    mtm.addTime(10000);
    mtm.addTime(10000);
    mtm.addTime(10000);
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);
    mam.current_auction_test.starting_time = 10;
    ao.sanityChecks();
    EXPECT_TRUE(ao.error_3);
}

TEST(TestAuctionObserver, testStartOneAuctionIfInStateAuctioning)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    mam.participating_test = false;
    MockTimeManager mtm;
    rsm.setRobotState(robot_state::AUCTIONING);
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);
    mam.current_auction_test.starting_time = 10;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_TRUE(mam.auction_started_test);
    mam.auction_started_test = false;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_FALSE(mam.auction_started_test);
}

TEST(TestAuctionObserver, testStartOneAuctionWhenAgainInStateAuctioning)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    mam.participating_test = false;
    MockTimeManager mtm;
    rsm.setRobotState(robot_state::AUCTIONING);
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);
    mam.current_auction_test.starting_time = 10;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_TRUE(mam.auction_started_test);
    mam.auction_started_test = false;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_FALSE(mam.auction_started_test);

    rsm.setRobotState(robot_state::MOVING_TO_FRONTIER);
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_FALSE(mam.auction_started_test);

    rsm.setRobotState(robot_state::AUCTIONING);
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_TRUE(mam.auction_started_test);
}

TEST(TestAuctionObserver, testCharging)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    MockTimeManager mtm;
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);

    mam.winner_test = true;
    mam.participating_test = false;
    rsm.setRobotState(robot_state::CHARGING);
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::CHARGING, rsm.getRobotState());

    mam.winner_test = false;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::CHARGING_ABORTED, rsm.getRobotState());
}

TEST(TestAuctionObserver, testFromInQueueToGoingCheckingVacancy)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    mam.participating_test = false;
    MockTimeManager mtm;
    rsm.setRobotState(robot_state::IN_QUEUE);
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);

    mam.winner_test = false;
    mtm.addTime(90 + reauctioning_timeout / 2); //TODO load reauctioning_time 
    mam.current_auction_test.ending_time = 90;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::IN_QUEUE, rsm.getRobotState());
    EXPECT_FALSE(mam.auction_started_test);

    ros::NodeHandle nh;
    ros::Publisher new_optimal_ds_pub = nh.advertise<adhoc_communication::EmDockingStation>("explorer/new_optimal_ds", 10);
    adhoc_communication::EmDockingStation msg;

    msg.id = 9;
    new_optimal_ds_pub.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    mam.winner_test = true;
    mam.current_auction_test.docking_station_id = 9;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::GOING_CHECKING_VACANCY, rsm.getRobotState());

    rsm.setRobotState(robot_state::IN_QUEUE);
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::IN_QUEUE, rsm.getRobotState());
}

TEST(TestAuctionObserver, testReauctioning)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    mam.participating_test = false;
    MockTimeManager mtm;
    rsm.setRobotState(robot_state::IN_QUEUE);
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);

    mam.winner_test = false;
    mtm.addTime(90 + reauctioning_timeout / 2);
    mam.current_auction_test.ending_time = 90;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::IN_QUEUE, rsm.getRobotState());
    EXPECT_FALSE(mam.auction_started_test);

    mtm.addTime(90 + reauctioning_timeout * 2);
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::IN_QUEUE, rsm.getRobotState());
    EXPECT_TRUE(mam.auction_started_test);
}

TEST(TestAuctionObserver, testGoChargingOnlyIfNewAuctionWasWon)
{
    AuctionObserver ao;
    RobotStateManager2 rsm;
    MockAuctionManager mam;
    mam.participating_test = false;
    MockTimeManager mtm;
    ao.setRobotStateManager(&rsm);
    ao.setAuctionManager(&mam);
    ao.setTimeManager(&mtm);

    ros::NodeHandle nh;
    ros::Publisher new_optimal_ds_pub = nh.advertise<adhoc_communication::EmDockingStation>("explorer/new_optimal_ds", 10);
    adhoc_communication::EmDockingStation msg;

    msg.id = 9;
    new_optimal_ds_pub.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    rsm.setRobotState(robot_state::MOVING_TO_FRONTIER);
    mam.current_auction_test.auction_id = 1;
    mam.current_auction_test.docking_station_id = 9;
    mam.winner_test = true;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::MOVING_TO_FRONTIER, rsm.getRobotState());

    rsm.setRobotState(robot_state::CHOOSING_ACTION);
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::GOING_CHECKING_VACANCY, rsm.getRobotState());

    rsm.setRobotState(robot_state::CHOOSING_ACTION);
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::COMPUTING_NEXT_GOAL, rsm.getRobotState());

    rsm.setRobotState(robot_state::CHOOSING_ACTION);
    mam.current_auction_test.auction_id = 2;
    mam.winner_test = false;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::COMPUTING_NEXT_GOAL, rsm.getRobotState());

    rsm.setRobotState(robot_state::CHOOSING_ACTION);
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::COMPUTING_NEXT_GOAL, rsm.getRobotState());

    rsm.setRobotState(robot_state::CHOOSING_ACTION);
    mam.current_auction_test.auction_id = 3;
    mam.winner_test = true;
    mam.current_auction_test.docking_station_id = 8;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::COMPUTING_NEXT_GOAL, rsm.getRobotState());

    msg.id = 8;
    new_optimal_ds_pub.publish(msg);
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    rsm.setRobotState(robot_state::CHOOSING_ACTION);
    mam.current_auction_test.auction_id = 4;
    mam.winner_test = true;
    ao.actAccordingToRobotStateAndAuctionResult();
    EXPECT_EQ(robot_state::GOING_CHECKING_VACANCY, rsm.getRobotState());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "auction_observer_utest");
    ros::NodeHandle private_nh("~");
    private_nh.getParam("reauctioning_timeout", reauctioning_timeout);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
