#include <ros/ros.h>
#include <gtest/gtest.h>
#include <adhoc_communication/EmAuction.h>
#include <utilities/mock_time_manager.h>
#include "auction_manager.h"
#include "mock_bid_computer.h"


#define ROBOT_ID 5
#define SENDING_SLEEP_TIME 0.3

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
    EXPECT_FALSE(am.isRobotParticipatingToAuction());
}

TEST(TestAuctionManager, testAuctionStartAndConclusion)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    am.tryToAcquireDs();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());

    while(am.isRobotParticipatingToAuction())
        ros::spinOnce();

    EXPECT_FALSE(am.isRobotParticipatingToAuction());
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
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    am.tryToAcquireDs();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());

    while(am.isRobotParticipatingToAuction())
        ros::spinOnce();

    EXPECT_FALSE(am.isRobotParticipatingToAuction());
    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testVictoryWithTwoParticipatingRobots)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    ros::NodeHandle nh;
    ros::Publisher auction_reply_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_reply", 10);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 10 + ROBOT_ID;
    msg.bid = 50;
    msg.robot = 222;
    am.tryToAcquireDs();

    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    auction_reply_pub.publish(msg);

    while(am.isRobotParticipatingToAuction())
        ros::spinOnce();

    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testLoseWithTwoParticipatingRobots)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = ""; //TODO
    ros::Publisher auction_reply_pub = nh.advertise<adhoc_communication::EmAuction>(prefix + "adhoc_communication/send_em_auction/auction_reply", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 10 + ROBOT_ID;
    msg.bid = 500;
    msg.robot = 222;

    am.tryToAcquireDs();

    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    auction_reply_pub.publish(msg);

    while(am.isRobotParticipatingToAuction())
        ros::spinOnce();

    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testBidReception)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>(prefix + "adhoc_communication/send_em_auction/auction_starting", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    ros::spinOnce();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
}

TEST(TestAuctionManager, testEndAuctionStartedByAnotherRobot)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>(prefix + "adhoc_communication/send_em_auction/auction_starting", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    ros::spinOnce();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    while(am.isRobotParticipatingToAuction())
        ros::spinOnce();
    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testWinAuctionStartedByAnotherRobot)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_starting", 10, true);
    ros::Publisher auction_result_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_result", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 10;
    msg.robot = 222;

    EXPECT_FALSE(am.isRobotParticipatingToAuction());
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    ros::spinOnce();
    ros::Duration(SENDING_SLEEP_TIME).sleep();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());

    adhoc_communication::EmAuction msg2;
    msg.auction = 43;
    msg.robot = ROBOT_ID;
    auction_result_pub.publish(msg);

    while(am.isRobotParticipatingToAuction())
        ros::spinOnce();
    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testDoNotParticipateToAnotherRobotAuction)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    ros::NodeHandle nh;
    std::string prefix = "";
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_starting", 10, true);
    
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 500;
    msg.robot = 222;
    auction_starting_pub.publish(msg);
    ros::Duration(SENDING_SLEEP_TIME).sleep();

    ros::spinOnce();
    for(int i=0; i<5; i++) {
        ros::Duration(1).sleep();
        EXPECT_FALSE(am.isRobotParticipatingToAuction());
    }
    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testInterruptAuction)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());

    ros::NodeHandle nh;
    ros::Publisher auction_starting_pub = nh.advertise<adhoc_communication::EmAuction>("adhoc_communication/send_em_auction/auction_starting", 10, true);
    adhoc_communication::EmAuction msg;
    msg.auction = 43;
    msg.bid = 500;
    msg.starting_time = 20;
    msg.robot = 222;
    
    am.tryToAcquireDs();
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    EXPECT_TRUE(am.isRobotParticipatingToAuction());

    auction_starting_pub.publish(msg);
    ros::Duration(0.5).sleep();
    ros::spinOnce();

    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
    EXPECT_TRUE(am.isRobotParticipatingToAuction());
    while(am.isRobotParticipatingToAuction())
        ros::spinOnce();
    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
}

TEST(TestAuctionManager, testScheduleNextAuction)
{
    AuctionManager am(ROBOT_ID);
    MockBidComputer *mbd = new MockBidComputer();
    MockTimeManager *mtm = new MockTimeManager();
    am.setBidComputer(mbd);
    am.setTimeManager(mtm);
    mbd->addBid(100);
    mtm->addTime(10);

    am.scheduleNextAuction();

    while(!am.isRobotParticipatingToAuction())
        ros::spinOnce();

    while(am.isRobotParticipatingToAuction())
        ros::spinOnce();

    EXPECT_TRUE(am.isRobotWinnerOfMostRecentAuction());
}

int main(int argc, char **argv){
  ros::init(argc, argv, "energy_mgmt");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
