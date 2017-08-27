#include <ros/ros.h>
#include <gtest/gtest.h>
#include "auction_manager.h"
#include "concrete_sender.h"

TEST(TestAuctionManagerAndSender, test)
{
    AuctionManager am(123);
    ConcreteSender sender;

    EXPECT_FALSE(am.isRobotWinnerOfMostRecentAuction());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "auction_manager_and_sender_itest");
    ros::NodeHandle nh;
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
