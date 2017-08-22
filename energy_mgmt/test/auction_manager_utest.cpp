#include <ros/ros.h>
#include <gtest/gtest.h>
#include <adhoc_communication/EmAuction.h>
#include <utilities/mock_time_manager.h>
#include "auction_manager.h"
#include "mock_bid_computer.h"
#include "mock_sender.h"

#define ROBOT_ID 5
#define SENDING_SLEEP_TIME 0.3

ros::ServiceServer send_auction_ss;

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

TEST(TestAuctionManager, testNonParticipation)
{
    AuctionManager am(ROBOT_ID);
    am.lock();
    EXPECT_FALSE(am.isRobotParticipatingToAuction());
    am.unlock();
}

TEST(TestAuctionManager, testAuctionStartAndConclusion)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms); //TODO i should use it to check that the sent bid is correct (bid sent to indicate auction start and auction winner)
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(123);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(20); //TODO check everywhere that the ending time is correct

    am.lock();
    am.tryToAcquireDs();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();

    auction_t current_auction = am.getCurrentAuction();

    //TODO do these checks also in all other places
    EXPECT_EQ(15, current_auction.auction_id);
    EXPECT_EQ(10, current_auction.starting_time);
    EXPECT_EQ(-1, current_auction.ending_time);
    EXPECT_EQ(5, current_auction.auctioneer);
    EXPECT_EQ(123, current_auction.docking_station_id);

    bool loop;
    do {
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();
        ros::spinOnce();
    }
    while(loop); 

    am.lock();
    EXPECT_FALSE(am.isRobotParticipatingToAuction());
    am.unlock();

    current_auction = am.getCurrentAuction();
    EXPECT_EQ(20, current_auction.ending_time);
    EXPECT_EQ(5, current_auction.winner_robot);

    mtm->addTime(30);
    mtm->addTime(40);
    mbd->addBid(100);
    am.tryToAcquireDs();
    current_auction = am.getCurrentAuction();
    EXPECT_EQ(25, current_auction.auction_id);
}

//TEST(TestAuctionManager, testSleepingBetweenTwoAuctions) //TODO
//{
//    EXPECT_FALSE(true);
//}

TEST(TestAuctionManager, testIsRobotWinnerOfMostRecentAuction)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(123);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(10);

    am.tryToAcquireDs();
    am.lock();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();

    bool loop;
    do {
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();
        ros::spinOnce();
    }
    while(loop);

    am.lock();
    EXPECT_FALSE(am.isRobotParticipatingToAuction());
    am.unlock();
    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction()); //TODO shoud lock everywhere...
}

TEST(TestAuctionManager, testVictoryWithTwoParticipatingRobots)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(123);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(10);

    ros::NodeHandle nh;
    ros::Publisher auction_reply_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_reply", 10);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 10 + ROBOT_ID;
    msg.bid = 50;
    msg.robot = 222;
    msg.docking_station = 123;
    am.tryToAcquireDs();

    am.lock();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();
    auction_reply_pub.publish(msg);

    bool loop;
    do {
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();
        ros::spinOnce();
    }
    while(loop);

    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testOptimalDsNotSet)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>(prefix + "adhoc_communication/send_em_auction/auction_starting", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    msg.docking_station = 7;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    for(unsigned int i=0; i<5; i++) {
        ros::spinOnce();
        am.lock();        
        EXPECT_FALSE(am.isRobotParticipatingToAuction());        
        am.unlock();
        ros::Duration(1).sleep();
    }
}

TEST(TestAuctionManager, testLoseWithTwoParticipatingRobots)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(123);
    mbd->addBid(100);
    mtm->addTime(888);
    mtm->addTime(999);

    ros::NodeHandle nh;
    std::string prefix = ""; //TODO
    ros::Publisher auction_reply_pub = nh.advertise<adhoc_communication::EmAuction>(prefix + "adhoc_communication/send_em_auction/auction_reply", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 10 + ROBOT_ID;
    msg.bid = 500;
    msg.robot = 222;
    msg.docking_station = 123;

    am.tryToAcquireDs();

    am.lock();
    auction_t auction = am.getCurrentAuction();
    EXPECT_EQ(10 + ROBOT_ID, auction.auction_id);
    EXPECT_EQ(888, auction.starting_time);
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();

    auction_reply_pub.publish(msg);
    bool loop;
    do {
        ros::spinOnce();
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();        
    }
    while(loop);

    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
    auction = am.getCurrentAuction();
    EXPECT_EQ(999, auction.ending_time);
}

TEST(TestAuctionManager, testIgnoreWrongBid)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(123);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = ""; //TODO
    ros::Publisher auction_reply_pub = nh.advertise<adhoc_communication::EmAuction>(prefix + "adhoc_communication/send_em_auction/auction_reply", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 20 + ROBOT_ID;
    msg.bid = 500;
    msg.robot = 222;
    msg.docking_station = 123;

    am.tryToAcquireDs();

    am.lock();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();

    auction_reply_pub.publish(msg);

    bool loop;
    do {
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();
        ros::spinOnce();
    }
    while(loop);

    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testBidReception)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(7);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>(prefix + "adhoc_communication/send_em_auction/auction_starting", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    msg.docking_station = 7;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    ros::spinOnce();
    am.lock();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();
}

TEST(TestAuctionManager, testEndAuctionStartedByAnotherRobot)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(78);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>(prefix + "adhoc_communication/send_em_auction/auction_starting", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    msg.docking_station = 78;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    ros::spinOnce();
    am.lock();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();
    bool loop;
    do {
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();
        ros::spinOnce();
    }
    while(loop);
    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testWinAuctionStartedByAnotherRobot)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(432);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_starting", 10, true);
    ros::Publisher auction_result_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_result", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    msg.docking_station = 432;

    am.lock();
    EXPECT_FALSE(am.isRobotParticipatingToAuction());
    am.unlock();
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    ros::spinOnce();
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    am.lock();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();

    adhoc_communication::EmAuction msg2;
    msg.auction = 43;
    msg.robot = ROBOT_ID;
    auction_result_pub.publish(msg);

    bool loop;
    do {
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();
        ros::spinOnce();
    }
    while(loop);
    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testDoNotParticipateToAnotherRobotAuction)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mbd->addBid(100);
    mbd->addBid(100);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(20);
    mtm->addTime(30);
    mtm->addTime(40);
    mtm->addTime(10);
    mtm->addTime(20);
    mtm->addTime(30);
    mtm->addTime(40);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_starting", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    msg.docking_station = 567;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    ros::spinOnce();
    for(int i=0; i<5; i++) {
        ros::Duration(1).sleep();
        am.lock();
        EXPECT_FALSE(am.isRobotParticipatingToAuction());
        am.unlock();
        ros::spinOnce();
    }

    am.setOptimalDs(567);

    msg.auction = 43;
    msg.bid = 500;
    msg.robot = 222;
    msg.docking_station = 567;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    ros::spinOnce();
    for(int i=0; i<5; i++) {
        ros::Duration(1).sleep();
        am.lock();
        EXPECT_FALSE(am.isRobotParticipatingToAuction());
        am.unlock();
        ros::spinOnce();
    }
    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());

    EXPECT_FALSE(am.isRobotParticipatingToAuction());

    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    msg.docking_station = 567;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    ros::spinOnce();
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    ros::spinOnce();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    bool loop;
    do {
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();
        ros::spinOnce();
    }
    while(loop);

    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    msg.docking_station = 0;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    ros::spinOnce();
    for(int i=0; i<5; i++) {
        ros::Duration(1).sleep();
        am.lock();
        EXPECT_FALSE(am.isRobotParticipatingToAuction());
        am.unlock();
        ros::spinOnce();
    }
    EXPECT_FALSE(am.isRobotParticipatingToAuction());
    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testInterruptAuction)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    MockSender *ms = new MockSender();
    am.setSender(ms);
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    am.setOptimalDs(7);
    mbd->addBid(100);
    mbd->addBid(100);
    mtm->addTime(10);
    mtm->addTime(20);
    mtm->addTime(10);
    mtm->addTime(20);

    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());

    ros::NodeHandle nh;
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_starting", 10, true);
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 500;
    msg.starting_time = 20;
    msg.robot = 222;
    msg.docking_station = 7;
    
    am.lock();
    am.tryToAcquireDs();
    ros::Duration(0.5).sleep();     
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();

    ros::spinOnce();

    auction_starting_pub.publish(msg);
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    am.lock();
    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());    
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    am.unlock();

    bool loop;
    do {
        am.lock();
        loop = am.isRobotParticipatingToAuction();
        am.unlock();
        ros::spinOnce();
    }
    while(loop);
    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
}

//TEST(TestAuctionManager, testScheduleNextAuction)
//{
//    AuctionManager am(ROBOT_ID);
//    MockBidComputer *mbd = new MockBidComputer();
//    MockTimeManager *mtm = new MockTimeManager();
//    MockSender *ms = new MockSender();
//    am.setSender(ms);
//    am.setBidComputer(mbd);
//    am.setTimeManager(mtm);
//    mbd->addBid(100);
//    mtm->addTime(10);
//    mtm->addTime(10);

//    am.scheduleNextAuction();

//    bool loop;
//    do {
//        am.lock();
//        loop = !am.isRobotParticipatingToAuction();
//        am.unlock();
//        ros::spinOnce();
//    }
//    while(loop);
//    do {
//        am.lock();
//        loop = am.isRobotParticipatingToAuction();
//        am.unlock();
//        ros::spinOnce();
//    }
//    while(loop);

//    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction());
//}

int main(int argc, char **argv){
    ros::init(argc, argv, "auction_manager_utest");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
